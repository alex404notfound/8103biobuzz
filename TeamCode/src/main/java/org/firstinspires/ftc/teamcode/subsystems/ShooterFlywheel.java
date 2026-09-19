package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

/** Two equally geared motors driving ONE mechanically linked flywheel.
 * Both receive the same logical voltage; both encoders must agree and reach speed.
 * Software PID + feedforward exclusively owns power (no second Hub velocity loop).
 */
@Config
public class ShooterFlywheel {
    public static volatile String leftMotorName = "launcherLeft", rightMotorName = "launcherRight";
    public static volatile boolean leftReversed = false, rightReversed = false, directionsVerified = false;
    public static final double TICKS_PER_REVOLUTION = 28, MAX_MOTOR_RPM = 6000;
    // Volts, motor-shaft RPM and seconds. kV is an initial 12 V / 6000 RPM estimate only.
    public static volatile double kP = 0.002, kI = 0, kD = 0, kS = 0, kV = 12.0 / 6000, kA = 0;
    public static volatile double maxVoltage = 3, testVoltage = 1, maxAccelerationRpmPerSecond = 1000;
    public static volatile double integralLimitRpmSeconds = 5000;
    public static volatile double rpmTolerance = 150, speedDwellMs = 250, spinupTimeoutMs = 4000;
    public static volatile double maxEncoderDifferenceRpm = 250, encoderMismatchDwellMs = 300;
    public static volatile double encoderResponseTimeoutMs = 1000, minimumEncoderRpm = 50;
    public static volatile double maximumSampleGapMs = 250;

    public enum Mode { OFF, VELOCITY, VOLTAGE, LEFT_TEST, RIGHT_TEST, FAULT }
    private final DcMotorEx left, right;
    private final DoubleSupplier voltageSource;
    private final LongSupplier clock;
    private final boolean initialLeftReversed, initialRightReversed;
    private Mode mode = Mode.OFF;
    private boolean enabled, closed, sampled, referenceInitialized, inBand, outputLimited;
    private double leftRpm, rightRpm, batteryVolts = Double.NaN, targetRpm, voltageRequest;
    private double referenceRpm, referenceAcceleration, feedbackVolts, feedforwardVolts;
    private double requestedVolts, appliedVolts, power, integral, previousAverage;
    private double[] appliedGains;
    private long sampleTime, controlTime = Long.MIN_VALUE, atSpeedSince = Long.MIN_VALUE;
    private long outsideSpeedSince = Long.MIN_VALUE, mismatchSince = Long.MIN_VALUE;
    private long missingLeftSince = Long.MIN_VALUE, missingRightSince = Long.MIN_VALUE;
    private String status = "INIT: encoder readout only";

    public ShooterFlywheel(HardwareMap map) {
        this(map.get(DcMotorEx.class, leftMotorName), map.get(DcMotorEx.class, rightMotorName),
                () -> readBattery(map), System::nanoTime);
    }

    ShooterFlywheel(DcMotorEx left, DcMotorEx right, DoubleSupplier voltageSource, LongSupplier clock) {
        this.left = left; this.right = right; this.voltageSource = voltageSource; this.clock = clock;
        initialLeftReversed = leftReversed; initialRightReversed = rightReversed;
        try {
            zeroMotors();
            left.setDirection(initialLeftReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            right.setDirection(initialRightReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (RuntimeException failure) {
            try { zeroMotors(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    public void enable() { if (!closed) enabled = true; }

    public void requestRpm(double rpm) {
        if (!enabled || closed || mode == Mode.FAULT) return;
        if (!Double.isFinite(rpm) || rpm < 0 || rpm > MAX_MOTOR_RPM) { fault("RPM target must be 0..6000"); return; }
        if (rpm == 0) { idle(); return; }
        if (mode != Mode.VELOCITY) enterMode(Mode.VELOCITY);
        if (rpm != targetRpm) {
            integral = 0; inBand = false; atSpeedSince = Long.MIN_VALUE;
            // Do not reset encoder-response/mismatch watchdogs on a live target change.
        }
        targetRpm = rpm;
    }

    /** Shared open-loop voltage for kS/kV measurements. Requires verified directions. */
    public void requestVoltage(double volts) {
        if (!enabled || closed || mode == Mode.FAULT) return;
        if (!Double.isFinite(volts) || volts < 0 || volts > 12) { fault("Voltage request must be 0..12 V"); return; }
        if (volts == 0) { idle(); return; }
        if (mode != Mode.VOLTAGE) enterMode(Mode.VOLTAGE);
        voltageRequest = volts;
    }

    /** The other motor coasts; its shaft/encoder can still turn because the motors are linked. */
    public void requestMotorTest(boolean testLeft) {
        if (!enabled || closed || mode == Mode.FAULT) return;
        Mode requested = testLeft ? Mode.LEFT_TEST : Mode.RIGHT_TEST;
        if (mode != requested) enterMode(requested);
    }

    private void enterMode(Mode selected) {
        zeroMotors(); resetControl(); mode = selected;
    }

    /** Release is the acknowledgement for a latched fault; never ramps down on release. */
    public void idle() {
        if (closed) return;
        status = mode == Mode.FAULT ? "Stopped after: " + status : "OFF (coasting)";
        mode = Mode.OFF; targetRpm = 0; resetControl(); zeroMotors();
    }

    public void periodic() {
        if (closed) return;
        try {
            long previousSample = sampleTime;
            boolean hadSample = sampled;
            leftRpm = ticksToRpm(left.getVelocity());
            rightRpm = ticksToRpm(right.getVelocity());
            batteryVolts = voltageSource.getAsDouble();
            sampleTime = clock.getAsLong(); // Include blocking read time in the stale-loop guard.
            sampled = true;
            if (!Double.isFinite(leftRpm) || !Double.isFinite(rightRpm)
                    || !Double.isFinite(batteryVolts) || batteryVolts < 6 || batteryVolts > 16.8) {
                fault("Invalid encoder or battery reading (battery must be 6..16.8 V)"); return;
            }
            if (!enabled || mode == Mode.OFF || mode == Mode.FAULT) { zeroMotors(); return; }
            if (!validConfiguration()) { fault("Invalid flywheel gains, limits or watchdog settings"); return; }
            if (initialLeftReversed != leftReversed || initialRightReversed != rightReversed) {
                fault("Motor direction changed; STOP and INIT to apply it"); return;
            }
            if (hadSample && (sampleTime < previousSample
                    || (sampleTime - previousSample) * 1e-6 > maximumSampleGapMs)) {
                fault("Control loop gap: release, check loop timing, then retry"); return;
            }
            boolean individual = mode == Mode.LEFT_TEST || mode == Mode.RIGHT_TEST;
            if (!individual && !directionsVerified) { fault("Verify both motor/encoder directions first"); return; }
            if (!individual && (leftRpm < -minimumEncoderRpm || rightRpm < -minimumEncoderRpm)) {
                fault("Encoder runs backward; verify both directions before RPM control"); return;
            }
            double average = (leftRpm + rightRpm) / 2;
            double dt = controlTime == Long.MIN_VALUE ? 0 : (sampleTime - controlTime) * 1e-9;
            controlTime = sampleTime;
            if (mode == Mode.VELOCITY) updateVelocity(average, dt);
            else {
                feedbackVolts = feedforwardVolts = 0;
                requestedVolts = individual ? testVoltage : voltageRequest;
            }
            if (mode == Mode.FAULT) return;
            if (!Double.isFinite(requestedVolts)) { fault("Nonfinite controller voltage"); return; }
            double ceiling = Math.min(maxVoltage, batteryVolts);
            appliedVolts = clamp(requestedVolts, 0, ceiling); // Flywheel never actively reverses.
            outputLimited = requestedVolts < 0 || requestedVolts > ceiling;
            power = appliedVolts / batteryVolts;
            if (!checkEncoders(individual)) return;
            if (mode == Mode.VELOCITY) {
                updateReadiness();
                if (mode == Mode.FAULT) return;
            }
            left.setPower(mode == Mode.RIGHT_TEST ? 0 : power);
            right.setPower(mode == Mode.LEFT_TEST ? 0 : power);
            previousAverage = average;
            status = mode == Mode.VELOCITY ? (isAtSpeed() ? "Both encoders at speed" : "Ramping / recovering speed")
                    : individual ? "Single motor direction test (linked shafts can both turn)" : "Shared voltage characterization";
        } catch (RuntimeException failure) {
            try { fault("Hardware exception; check connections before retrying"); }
            catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    private void updateVelocity(double average, double dt) {
        if (targetRpm <= rpmTolerance) { fault("Target RPM must exceed rpmTolerance"); return; }
        double[] gains = { kP, kI, kD, kS, kV, kA, integralLimitRpmSeconds };
        if (!Arrays.equals(appliedGains, gains)) { integral = 0; appliedGains = gains; }
        if (!referenceInitialized) {
            referenceRpm = Math.max(0, Math.min(MAX_MOTOR_RPM, average));
            previousAverage = average; referenceInitialized = true;
        }
        double previousReference = referenceRpm;
        double change = maxAccelerationRpmPerSecond * dt;
        referenceRpm += clamp(targetRpm - referenceRpm, -change, change);
        referenceAcceleration = dt > 0 ? (referenceRpm - previousReference) / dt : 0;
        double error = referenceRpm - average;
        double measuredAcceleration = dt > 0 ? (average - previousAverage) / dt : 0;
        feedforwardVolts = (referenceRpm > 0 ? kS : 0) + kV * referenceRpm + kA * referenceAcceleration;
        double pd = kP * error - kD * measuredAcceleration;
        double candidate = clamp(integral + error * dt, -integralLimitRpmSeconds, integralLimitRpmSeconds);
        double raw = pd + kI * candidate + feedforwardVolts;
        double ceiling = Math.min(maxVoltage, batteryVolts);
        if ((raw >= 0 && raw <= ceiling) || (raw > ceiling && error < 0) || (raw < 0 && error > 0)) integral = candidate;
        feedbackVolts = pd + kI * integral;
        requestedVolts = feedbackVolts + feedforwardVolts;
    }

    private void updateReadiness() {
        boolean rampFinished = referenceRpm == targetRpm;
        boolean bothInBand = rampFinished && Math.abs(leftRpm - targetRpm) <= rpmTolerance
                && Math.abs(rightRpm - targetRpm) <= rpmTolerance
                && Math.abs(leftRpm - rightRpm) <= maxEncoderDifferenceRpm;
        if (bothInBand) {
            if (!inBand) atSpeedSince = sampleTime;
            inBand = true; outsideSpeedSince = Long.MIN_VALUE;
        } else {
            inBand = false; atSpeedSince = Long.MIN_VALUE;
            // A deliberate ramp is allowed its own time. Once final speed is requested,
            // allow a bounded spin-up/recovery window; encoder faults remain active throughout.
            if (!rampFinished) outsideSpeedSince = Long.MIN_VALUE;
            else if (outsideSpeedSince == Long.MIN_VALUE) outsideSpeedSince = sampleTime;
            if (outsideSpeedSince != Long.MIN_VALUE && elapsedMs(outsideSpeedSince) >= spinupTimeoutMs) {
                fault("Speed timeout: check feedforward, voltage cap, load and encoders");
            }
        }
    }

    private boolean checkEncoders(boolean individual) {
        if (!individual && Math.abs(leftRpm - rightRpm) > maxEncoderDifferenceRpm) {
            if (mismatchSince == Long.MIN_VALUE) mismatchSince = sampleTime;
            if (elapsedMs(mismatchSince) >= encoderMismatchDwellMs) {
                fault("Linked motor encoders disagree; check encoders, gearing and directions"); return false;
            }
        } else mismatchSince = Long.MIN_VALUE;
        boolean substantialPower = appliedVolts >= 0.5;
        boolean checkLeft = substantialPower && mode != Mode.RIGHT_TEST;
        boolean checkRight = substantialPower && mode != Mode.LEFT_TEST;
        missingLeftSince = missingSince(checkLeft, Math.abs(leftRpm), missingLeftSince);
        missingRightSince = missingSince(checkRight, Math.abs(rightRpm), missingRightSince);
        if ((missingLeftSince != Long.MIN_VALUE && elapsedMs(missingLeftSince) >= encoderResponseTimeoutMs)
                || (missingRightSince != Long.MIN_VALUE && elapsedMs(missingRightSince) >= encoderResponseTimeoutMs)) {
            fault("No encoder response under power; release and check motor/encoder wiring or obstruction"); return false;
        }
        return true;
    }

    private long missingSince(boolean powered, double rpm, long previous) {
        return powered && rpm < minimumEncoderRpm ? (previous == Long.MIN_VALUE ? sampleTime : previous) : Long.MIN_VALUE;
    }

    private boolean validConfiguration() {
        return nonnegative(kP) && nonnegative(kI) && nonnegative(kD) && nonnegative(kS) && nonnegative(kV) && nonnegative(kA)
                && positive(maxVoltage) && maxVoltage <= 12 && nonnegative(testVoltage) && testVoltage <= 1.8
                && positive(maxAccelerationRpmPerSecond) && nonnegative(integralLimitRpmSeconds)
                && positive(rpmTolerance) && nonnegative(speedDwellMs) && positive(spinupTimeoutMs)
                && positive(maxEncoderDifferenceRpm) && positive(encoderMismatchDwellMs)
                && positive(encoderResponseTimeoutMs) && positive(minimumEncoderRpm)
                && positive(maximumSampleGapMs) && maximumSampleGapMs <= 250;
    }

    public boolean hasFreshSample() {
        long age = clock.getAsLong() - sampleTime;
        return sampled && age >= 0 && positive(maximumSampleGapMs) && age * 1e-6 <= maximumSampleGapMs;
    }

    public boolean isAtSpeed() {
        return enabled && !closed && mode == Mode.VELOCITY && directionsVerified && validConfiguration()
                && hasFreshSample() && inBand && referenceRpm == targetRpm
                && Math.abs(leftRpm - targetRpm) <= rpmTolerance && Math.abs(rightRpm - targetRpm) <= rpmTolerance
                && Math.abs(leftRpm - rightRpm) <= maxEncoderDifferenceRpm
                && (clock.getAsLong() - atSpeedSince) * 1e-6 >= speedDwellMs;
    }

    private void fault(String reason) {
        mode = Mode.FAULT; status = reason; resetControl(); zeroMotors();
    }

    private void resetControl() {
        referenceInitialized = inBand = outputLimited = false;
        referenceRpm = referenceAcceleration = feedbackVolts = feedforwardVolts = requestedVolts = integral = 0;
        controlTime = atSpeedSince = outsideSpeedSince = mismatchSince = missingLeftSince = missingRightSince = Long.MIN_VALUE;
    }

    private void zeroMotors() {
        power = appliedVolts = 0;
        try { left.setPower(0); } finally { right.setPower(0); }
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
    public static double rpmToTicks(double rpm) { return rpm * TICKS_PER_REVOLUTION / 60; }
    public static double ticksToRpm(double ticks) { return ticks * 60 / TICKS_PER_REVOLUTION; }
    public Mode getMode() { return mode; }
    public String getStatus() { return status; }
    public double getLeftRpm() { return leftRpm; }
    public double getRightRpm() { return rightRpm; }
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
