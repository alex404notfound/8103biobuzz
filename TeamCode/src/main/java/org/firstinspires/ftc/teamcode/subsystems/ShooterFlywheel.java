package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

/** One independently controlled flywheel motor with its own encoder and voltage-based PID/feedforward. */
public class ShooterFlywheel {
    public static final double MAX_MOTOR_RPM = 6000;
    public static final SpeedTuning SMALL_SHOOTER = new SpeedTuning();
    private static final long DIRECTION_SETTLE_NANOS = 400_000_000L;
    private static final double DEFAULT_RPM_TOLERANCE = 150;

    /** Shooter setpoint, motor direction, and gains in volts per RPM. */
    public static final class SpeedTuning {
        public volatile double setpointRpm = 0;
        public volatile double kV, kP;
        public volatile boolean reversed;

        public SpeedTuning() { this(0.015, 0.0016, true); }

        public SpeedTuning(double kP, double kV, boolean reversed) {
            this.kP = kP;
            this.kV = kV;
            this.reversed = reversed;
        }
    }

    /** Launcher exposes separate Panels settings for the big and small shooters. */
    public static class Settings {
        public volatile String motorName;
        public volatile boolean reversed = false, directionVerified = false;
        public volatile double ticksPerRevolution = 28;
        // Volts, motor-shaft RPM and seconds. Initial estimates, not measured gains.
        public volatile double kP = 0.002, kI = 0, kD = 0, kS = 0, kV = 12.0 / 6000, kA = 0;
        public volatile double maxVoltage = 3, testVoltage = 1, maxAccelerationRpmPerSecond = 1000;
        public volatile double integralLimitRpmSeconds = 5000;
        public volatile double rpmTolerance = DEFAULT_RPM_TOLERANCE, speedDwellMs = 250, spinupTimeoutMs = 4000;
        public volatile double encoderResponseTimeoutMs = 1000, minimumEncoderRpm = 50;
        public volatile double maximumSampleGapMs = 250;
        public Settings(String motorName) { this.motorName = motorName; }

        Settings(String motorName, SpeedTuning tuning) {
            this(motorName);
            kV = tuning.kV;
            kP = tuning.kP;
            reversed = tuning.reversed;
        }
    }

    public enum Mode { OFF, VELOCITY, FEEDFORWARD, VOLTAGE, DIRECTION_TEST, FAULT }
    private final DcMotorEx motor;
    private final Settings settings;
    private final DoubleSupplier voltageSource;
    private final LongSupplier clock;
    private final String initialName;
    private final double initialTicksPerRevolution;
    private boolean appliedReversed;
    private long directionAppliedAt = Long.MIN_VALUE;
    private Mode mode = Mode.OFF;
    private boolean enabled, closed, sampled, referenceInitialized, inBand, outputLimited;
    private double measuredRpm, batteryVolts = Double.NaN, targetRpm, voltageRequest;
    private double referenceRpm, referenceAcceleration, feedbackVolts, feedforwardVolts;
    private double requestedVolts, appliedVolts, power, integral, previousMeasuredRpm;
    private double[] appliedGains;
    private long sampleTime, controlTime = Long.MIN_VALUE, atSpeedSince = Long.MIN_VALUE;
    private long outsideSpeedSince = Long.MIN_VALUE, missingEncoderSince = Long.MIN_VALUE;
    private String status = "INIT: encoder readout only";

    ShooterFlywheel(DcMotorEx motor, Settings settings, HardwareMap map) {
        this(motor, settings, () -> readBattery(map), System::nanoTime);
    }

    public static ShooterFlywheel forSmallShooter(DcMotorEx motor, HardwareMap map) {
        return forSmallShooter(motor, map, DEFAULT_RPM_TOLERANCE);
    }

    public static ShooterFlywheel forSmallShooter(DcMotorEx motor, HardwareMap map, double rpmTolerance) {
        return forShooter(motor, map, "smallShooter", SMALL_SHOOTER, rpmTolerance);
    }

    public static ShooterFlywheel forTuning(DcMotorEx motor, HardwareMap map, String motorName, SpeedTuning tuning) {
        return forShooter(motor, map, motorName, tuning, DEFAULT_RPM_TOLERANCE);
    }

    private static ShooterFlywheel forShooter(DcMotorEx motor, HardwareMap map, String motorName,
                                              SpeedTuning tuning, double rpmTolerance) {
        Settings settings = new Settings(motorName, tuning);
        settings.directionVerified = true;
        settings.maxVoltage = 12;
        settings.maxAccelerationRpmPerSecond = MAX_MOTOR_RPM;
        settings.rpmTolerance = rpmTolerance;
        ShooterFlywheel flywheel = new ShooterFlywheel(motor, settings, map);
        flywheel.enable();
        return flywheel;
    }

    ShooterFlywheel(DcMotorEx motor, Settings settings, DoubleSupplier voltageSource, LongSupplier clock) {
        this.motor = motor; this.settings = settings; this.voltageSource = voltageSource; this.clock = clock;
        appliedReversed = settings.reversed;
        initialName = settings.motorName;
        initialTicksPerRevolution = settings.ticksPerRevolution;
        try {
            zeroMotors();
            if (initialName == null || initialName.trim().isEmpty() || !positive(initialTicksPerRevolution))
                throw new IllegalArgumentException("Set a motor name and positive encoder ticks per revolution");
            motor.setDirection(appliedReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (RuntimeException failure) {
            try { zeroMotors(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    public void enable() { if (!closed) enabled = true; }

    /** Apply a live tuning direction and wait for the flywheel to brake before resuming. */
    public boolean prepareTuningDirection(boolean reversed) {
        if (closed) return false;
        if (reversed != appliedReversed) {
            idle();
            motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            appliedReversed = settings.reversed = reversed;
            directionAppliedAt = clock.getAsLong();
        }
        if (directionAppliedAt == Long.MIN_VALUE) return true;
        if (clock.getAsLong() - directionAppliedAt < DIRECTION_SETTLE_NANOS
                || Math.abs(ticksToRpm(motor.getVelocity())) > settings.minimumEncoderRpm) return false;
        directionAppliedAt = Long.MIN_VALUE;
        return true;
    }

    /** Apply live gains and take one control step. Call each loop while requesting speed. */
    public void updateRpm(double rpm, SpeedTuning tuning) {
        updateSpeed(rpm, tuning.kV, tuning.kP, Mode.VELOCITY, false);
    }

    /** Tune the voltage feedforward on its own when the feedback gain is zero. */
    public void updateTuningRpm(double rpm, SpeedTuning tuning) {
        double kV = tuning.kV;
        double kP = tuning.kP;
        if (rpm == 0 || (kV == 0 && kP == 0)) {
            idle();
            return;
        }
        // Setting kP to zero is a deliberate switch to tuning without feedback.
        if (mode == Mode.FAULT && kP == 0 && settings.kP != 0) idle();
        updateSpeed(rpm, kV, kP, kP == 0 ? Mode.FEEDFORWARD : Mode.VELOCITY, true);
    }

    private void updateSpeed(double rpm, double kV, double kP, Mode requestedMode, boolean tuning) {
        if (mode == Mode.OFF) sampled = false; // Start a fresh sample window after idle.
        settings.kV = kV;
        settings.kP = kP;
        requestSpeed(rpm, requestedMode);
        periodic(tuning);
    }

    public void requestRpm(double rpm) {
        requestSpeed(rpm, Mode.VELOCITY);
    }

    private void requestSpeed(double rpm, Mode requestedMode) {
        if (!enabled || closed || mode == Mode.FAULT) return;
        if (!Double.isFinite(rpm) || rpm < 0 || rpm > MAX_MOTOR_RPM) { fault("RPM target must be 0..6000"); return; }
        if (rpm == 0) { idle(); return; }
        if (mode != requestedMode) enterMode(requestedMode);
        if (rpm != targetRpm) {
            integral = 0; inBand = false; atSpeedSince = Long.MIN_VALUE;
            // Do not reset the encoder-response watchdog on a live target change.
        }
        targetRpm = rpm;
    }

    /** Open-loop voltage for kS/kV measurements. Requires a verified motor direction. */
    public void requestVoltage(double volts) {
        if (!enabled || closed || mode == Mode.FAULT) return;
        if (!Double.isFinite(volts) || volts < 0 || volts > 12) { fault("Voltage request must be 0..12 V"); return; }
        if (volts == 0) { idle(); return; }
        if (mode != Mode.VOLTAGE) enterMode(Mode.VOLTAGE);
        voltageRequest = volts;
    }

    /** Low-voltage direction/encoder check for this motor only. */
    public void requestMotorTest() {
        if (!enabled || closed || mode == Mode.FAULT) return;
        if (mode != Mode.DIRECTION_TEST) enterMode(Mode.DIRECTION_TEST);
    }

    private void enterMode(Mode selected) {
        zeroMotors(); resetControl(); mode = selected;
    }

    /** Release is the acknowledgement for a latched fault; never ramps down on release. */
    public void idle() {
        if (closed) return;
        status = mode == Mode.FAULT ? "Stopped after: " + status : "OFF (BRAKE)";
        mode = Mode.OFF; targetRpm = 0; resetControl(); zeroMotors();
    }

    public void periodic() {
        periodic(false);
    }

    private void periodic(boolean tuning) {
        if (closed) return;
        try {
            if (!hardwareConfigurationMatches()) {
                fault("Motor name, direction or encoder resolution changed; STOP and INIT"); return;
            }
            long previousSample = sampleTime;
            boolean hadSample = sampled;
            boolean feedforwardOnly = mode == Mode.FEEDFORWARD;
            // Each shooter uses its own encoder and independently tuned controller.
            measuredRpm = motor.getVelocity() * 60 / initialTicksPerRevolution;
            batteryVolts = voltageSource.getAsDouble();
            sampleTime = clock.getAsLong(); // Include blocking read time in the stale-loop guard.
            sampled = true;
            if ((!feedforwardOnly && !Double.isFinite(measuredRpm))
                    || !Double.isFinite(batteryVolts) || batteryVolts < 6 || batteryVolts > 16.8) {
                fault("Invalid encoder or battery reading (battery must be 6..16.8 V)"); return;
            }
            if (!enabled || mode == Mode.OFF || mode == Mode.FAULT) { zeroMotors(); return; }
            if (!validConfiguration()) { fault("Invalid flywheel gains, limits or watchdog settings"); return; }
            if (hadSample && (sampleTime < previousSample
                    || (sampleTime - previousSample) * 1e-6 > settings.maximumSampleGapMs)) {
                if (tuning) {
                    resetControl();
                    zeroMotors();
                    status = "Control loop gap. Waiting for the next sample";
                } else {
                    fault("Control loop gap: release, check loop timing, then retry");
                }
                return;
            }
            boolean individual = mode == Mode.DIRECTION_TEST;
            if (!individual && !settings.directionVerified) { fault("Verify this motor direction and its encoder first"); return; }
            if (!individual && !feedforwardOnly && measuredRpm < -settings.minimumEncoderRpm) {
                fault("Feedback encoder runs backward; verify directions before RPM control"); return;
            }
            double dt = controlTime == Long.MIN_VALUE ? 0 : (sampleTime - controlTime) * 1e-9;
            controlTime = sampleTime;
            if (mode == Mode.VELOCITY) updateVelocity(dt);
            else if (feedforwardOnly) {
                feedbackVolts = 0;
                feedforwardVolts = settings.kV * targetRpm;
                requestedVolts = feedforwardVolts;
            } else {
                feedbackVolts = feedforwardVolts = 0;
                requestedVolts = individual ? settings.testVoltage : voltageRequest;
            }
            if (mode == Mode.FAULT) return;
            if (!Double.isFinite(requestedVolts)) { fault("Nonfinite controller voltage"); return; }
            double ceiling = Math.min(settings.maxVoltage, batteryVolts);
            appliedVolts = clamp(requestedVolts, 0, ceiling); // Flywheel never actively reverses.
            outputLimited = requestedVolts < 0 || requestedVolts > ceiling;
            power = appliedVolts / batteryVolts;
            if (!feedforwardOnly && !checkEncoder()) return;
            if (mode == Mode.VELOCITY) {
                updateReadiness(tuning);
                if (mode == Mode.FAULT) return;
            }
            motor.setPower(power);
            previousMeasuredRpm = measuredRpm;
            if (feedforwardOnly) status = "Feedforward tuning";
            else status = mode == Mode.VELOCITY ? (isAtSpeed() ? "Flywheel at speed" : "Ramping / recovering speed")
                    : individual ? "Selected motor direction test" : "Selected motor voltage characterization";
        } catch (RuntimeException failure) {
            try { fault("Hardware exception; check connections before retrying"); }
            catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    private void updateVelocity(double dt) {
        if (targetRpm <= settings.rpmTolerance) { fault("Target RPM must exceed the speed tolerance"); return; }
        double[] gains = { settings.kP, settings.kI, settings.kD, settings.kS, settings.kV, settings.kA, settings.integralLimitRpmSeconds };
        if (!Arrays.equals(appliedGains, gains)) { integral = 0; appliedGains = gains; }
        if (!referenceInitialized) {
            referenceRpm = Math.max(0, Math.min(MAX_MOTOR_RPM, measuredRpm));
            previousMeasuredRpm = measuredRpm; referenceInitialized = true;
        }
        double previousReference = referenceRpm;
        double change = settings.maxAccelerationRpmPerSecond * dt;
        referenceRpm += clamp(targetRpm - referenceRpm, -change, change);
        referenceAcceleration = dt > 0 ? (referenceRpm - previousReference) / dt : 0;
        double error = referenceRpm - measuredRpm;
        double measuredAcceleration = dt > 0 ? (measuredRpm - previousMeasuredRpm) / dt : 0;
        feedforwardVolts = (referenceRpm > 0 ? settings.kS : 0) + settings.kV * referenceRpm + settings.kA * referenceAcceleration;
        double pd = settings.kP * error - settings.kD * measuredAcceleration;
        double candidate = clamp(integral + error * dt, -settings.integralLimitRpmSeconds, settings.integralLimitRpmSeconds);
        double raw = pd + settings.kI * candidate + feedforwardVolts;
        double ceiling = Math.min(settings.maxVoltage, batteryVolts);
        if ((raw >= 0 && raw <= ceiling) || (raw > ceiling && error < 0) || (raw < 0 && error > 0)) integral = candidate;
        feedbackVolts = pd + settings.kI * integral;
        requestedVolts = feedbackVolts + feedforwardVolts;
    }

    private void updateReadiness(boolean tuning) {
        boolean rampFinished = referenceRpm == targetRpm;
        boolean speedInBand = rampFinished && Math.abs(measuredRpm - targetRpm) <= settings.rpmTolerance;
        if (speedInBand) {
            if (!inBand) atSpeedSince = sampleTime;
            inBand = true; outsideSpeedSince = Long.MIN_VALUE;
        } else {
            inBand = false; atSpeedSince = Long.MIN_VALUE;
            // During shooting, allow a bounded recovery window after the ramp finishes.
            if (tuning || !rampFinished) outsideSpeedSince = Long.MIN_VALUE;
            else if (outsideSpeedSince == Long.MIN_VALUE) outsideSpeedSince = sampleTime;
            if (outsideSpeedSince != Long.MIN_VALUE && elapsedMs(outsideSpeedSince) >= settings.spinupTimeoutMs) {
                fault("Speed timeout: check feedforward, voltage cap, load and feedback encoder");
            }
        }
    }

    private boolean checkEncoder() {
        // A powered shooter must produce a response from its own encoder.
        boolean missing = appliedVolts >= 0.5 && Math.abs(measuredRpm) < settings.minimumEncoderRpm;
        if (missing && missingEncoderSince == Long.MIN_VALUE) missingEncoderSince = sampleTime;
        else if (!missing) missingEncoderSince = Long.MIN_VALUE;
        if (missingEncoderSince != Long.MIN_VALUE && elapsedMs(missingEncoderSince) >= settings.encoderResponseTimeoutMs) {
            fault("No feedback encoder response; check encoder, motor wiring or obstruction"); return false;
        }
        return true;
    }

    private boolean hardwareConfigurationMatches() {
        return appliedReversed == settings.reversed && initialName.equals(settings.motorName)
                && initialTicksPerRevolution == settings.ticksPerRevolution;
    }

    private boolean validConfiguration() {
        return nonnegative(settings.kP) && nonnegative(settings.kI) && nonnegative(settings.kD) && nonnegative(settings.kS) && nonnegative(settings.kV) && nonnegative(settings.kA)
                && positive(settings.maxVoltage) && settings.maxVoltage <= 12 && nonnegative(settings.testVoltage) && settings.testVoltage <= 1.8
                && positive(settings.maxAccelerationRpmPerSecond) && nonnegative(settings.integralLimitRpmSeconds)
                && positive(settings.rpmTolerance) && nonnegative(settings.speedDwellMs) && positive(settings.spinupTimeoutMs)
                && positive(settings.encoderResponseTimeoutMs) && positive(settings.minimumEncoderRpm)
                && positive(settings.maximumSampleGapMs) && settings.maximumSampleGapMs <= 250;
    }

    public boolean hasFreshSample() {
        long age = clock.getAsLong() - sampleTime;
        return sampled && hardwareConfigurationMatches() && age >= 0
                && positive(settings.maximumSampleGapMs) && age * 1e-6 <= settings.maximumSampleGapMs;
    }

    public boolean isWithinTargetSpeed() {
        return enabled && !closed && mode == Mode.VELOCITY && settings.directionVerified && validConfiguration()
                && hasFreshSample() && Math.abs(measuredRpm - targetRpm) <= settings.rpmTolerance;
    }

    public boolean isAtSpeed() {
        return isWithinTargetSpeed() && inBand && referenceRpm == targetRpm
                && (clock.getAsLong() - atSpeedSince) * 1e-6 >= settings.speedDwellMs;
    }

    private void fault(String reason) {
        mode = Mode.FAULT; status = reason; resetControl(); zeroMotors();
    }

    private void resetControl() {
        referenceInitialized = inBand = outputLimited = false;
        referenceRpm = referenceAcceleration = feedbackVolts = feedforwardVolts = requestedVolts = integral = 0;
        controlTime = atSpeedSince = outsideSpeedSince = missingEncoderSince = Long.MIN_VALUE;
    }

    private void zeroMotors() {
        power = appliedVolts = 0;
        motor.setPower(0);
    }

    public void stop() {
        if (closed) return;
        closed = true; enabled = false; mode = Mode.OFF; targetRpm = 0;
        status = "Stopped"; resetControl(); zeroMotors();
    }

    private static double readBattery(HardwareMap map) {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : map.voltageSensor) {
            double value = sensor.getVoltage();
            if (Double.isFinite(value) && value > 0) lowest = Math.min(lowest, value);
        }
        return lowest == Double.POSITIVE_INFINITY ? Double.NaN : lowest;
    }

    private double elapsedMs(long since) { return (sampleTime - since) * 1e-6; }
    private static double clamp(double n, double low, double high) { return Math.max(low, Math.min(high, n)); }
    private static boolean positive(double n) { return Double.isFinite(n) && n > 0; }
    private static boolean nonnegative(double n) { return Double.isFinite(n) && n >= 0; }
    public double rpmToTicks(double rpm) { return rpm * initialTicksPerRevolution / 60; }
    public double ticksToRpm(double ticks) { return ticks * 60 / initialTicksPerRevolution; }
    public Mode getMode() { return mode; }
    public String getStatus() { return status; }
    public double getMeasuredRpm() { return measuredRpm; }
    public String getFeedbackEncoder() { return initialName; }
    public double getTargetRpm() { return targetRpm; }
    public double getReferenceRpm() { return referenceRpm; }
    public double getReferenceAccelerationRpmPerSecond() { return referenceAcceleration; }
    public double getFeedbackVolts() { return feedbackVolts; }
    public double getFeedforwardVolts() { return feedforwardVolts; }
    public double getRequestedVolts() { return requestedVolts; }
    public double getAppliedVolts() { return appliedVolts; }
    public double getPower() { return power; }
    public double getBatteryVolts() { return batteryVolts; }
    public boolean isOutputLimited() { return outputLimited; }
}
