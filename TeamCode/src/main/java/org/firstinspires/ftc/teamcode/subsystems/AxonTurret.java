package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.math.ContinuousAngleTracker;
import org.firstinspires.ftc.teamcode.math.TurretMotionProfile;

import java.util.Arrays;
import java.util.function.LongSupplier;

/** CR-mode Axon plus analog shaft feedback. Call periodic every loop, including INIT.
 * Position is relative to an explicitly established zero, never a retained multi-turn absolute pose.
 * See docs/AXON_TURRET_TEST.md and the RTPAxon reference linked there.
 */
@Config
public class AxonTurret {
    // Dashboard writes on another thread; volatile keeps edits visible to the control loop.
    public static volatile String servoName = "turret", encoderName = "turretEncoder";
    // Sensor full-scale, NOT AnalogInput.getMaxVoltage() (the Hub input range may differ).
    public static volatile double encoderMinVolts = 0, encoderMaxVolts = 3.3;
    public static volatile double encoderSign = 1, servoSign = 1;
    public static volatile double servoTurnsPerTurretTurn = 0; // Measure; 20T driving 100T means 5.
    public static volatile double maximumServoDegreesPerSecond = 0; // Datasheet speed with margin; required.
    public static volatile double maximumSampleGapMs = 100;
    public static volatile boolean directionsVerified = false, limitTravel = true;
    public static volatile double minDegrees = 0, maxDegrees = 0; // Measured limits relative to chosen zero.
    public static volatile double kP = 0.01, kI = 0, kD = 0;
    // Feedforward uses normalized CR-servo power and TURRET degrees/seconds, not motor volts.
    public static volatile double kS = 0, kV = 0, kA = 0;
    public static volatile boolean profileEnabled = true;
    public static volatile double maxProfileVelocityDegreesPerSecond = 30;
    public static volatile double maxProfileAccelerationDegreesPerSecondSquared = 60;
    public static volatile double maximumPower = 0.15, manualPower = 0.10, integralLimit = 20;
    public static volatile double toleranceDegrees = 1, settledVelocityDegreesPerSecond = 3;
    public static volatile double settleTimeMs = 200, moveTimeoutMs = 4000; // Allowance beyond planned duration.
    public static volatile double noMotionTimeoutMs = 750, noMotionMinimumServoDegrees = 2;
    public static volatile double noMotionPowerThreshold = 0.05;

    public enum Mode { OFF, MANUAL, POSITION, FAULT }
    private final CRServo servo;
    private final AnalogInput encoder;
    private final LongSupplier clock;
    private ContinuousAngleTracker tracker;
    private double[] calibration;
    private boolean enabled, closed, referenced, zeroRequested, sensorValid;
    private Mode mode = Mode.OFF;
    private String status = "INIT: encoder readout only";
    private double volts = Double.NaN, wrappedDegrees = Double.NaN;
    private double zeroShaftDegrees, targetDegrees, manualRequest, output, integral;
    private long sampleTime, controlTime = Long.MIN_VALUE, moveStarted = Long.MIN_VALUE;
    private long settledSince = Long.MIN_VALUE, motionSince = Long.MIN_VALUE;
    private double motionAnchor, motionDirection;
    private TurretMotionProfile profile;
    private long profileStarted;
    private double profileGoal = Double.NaN, profileMaxVelocity, profileMaxAcceleration;
    private double desiredPosition = Double.NaN, desiredVelocity = Double.NaN, desiredAcceleration = Double.NaN;
    private double feedbackPower, feedforwardPower, unclampedPower, profileDuration;
    private boolean profileFinished, outputLimited;
    private double[] gains;

    public AxonTurret(HardwareMap map) {
        this(map.get(CRServo.class, servoName), map.get(AnalogInput.class, encoderName), System::nanoTime);
    }

    AxonTurret(CRServo servo, AnalogInput encoder, LongSupplier clock) {
        this.servo = servo;
        this.encoder = encoder;
        this.clock = clock;
        servo.setPower(0);
        servo.setDirection(DcMotorSimple.Direction.FORWARD); // Signs are explicit in our calibration.
    }

    public void enable() { if (!closed) enabled = true; }

    /** Fraction of manualPower. Before zero this is a supervised raw jog without travel limits. */
    public void requestManual(double fraction) {
        if (!enabled || closed || mode == Mode.FAULT) return;
        if (mode != Mode.MANUAL) resetControl();
        mode = Mode.MANUAL;
        manualRequest = fraction;
    }

    /** A bounded, UNWRAPPED turret angle. No shortest-path wrapping across cable limits. */
    public void requestPosition(double degrees) {
        if (!enabled || closed || mode == Mode.FAULT) return;
        if (mode != Mode.POSITION) resetControl();
        else if (degrees != targetDegrees) {
            // Preserve the running profile and motion watchdog during a retarget.
            integral = 0; controlTime = moveStarted = settledSince = Long.MIN_VALUE;
        }
        mode = Mode.POSITION;
        targetDegrees = degrees;
    }

    /** Release removes power immediately. A fault stays latched until clearFault is requested. */
    public void idle() {
        if (closed) return;
        if (mode != Mode.FAULT) mode = Mode.OFF;
        zeroRequested = false;
        resetControl();
        writePower(0);
    }

    public void clearFault() {
        if (!enabled || closed || mode != Mode.FAULT) return;
        idle();
        mode = Mode.OFF;
        status = "Fault cleared; establish zero again";
    }

    /** Operator must align the mechanism to its chosen physical zero and release all motion. */
    public void zeroHere() {
        if (enabled && !closed && mode == Mode.OFF) zeroRequested = true;
    }

    public void periodic() {
        if (closed) return;
        try {
            sample();
            if (!enabled || mode == Mode.FAULT || !sensorValid) { writePower(0); return; }
            if (zeroRequested) {
                zeroRequested = false;
                if (mode == Mode.OFF && positionConfigurationValid()
                        && Math.abs(tracker.getVelocityDegreesPerSecond()) <= settledVelocityDegreesPerSecond) {
                    zeroShaftDegrees = tracker.getDegrees();
                    referenced = true;
                    targetDegrees = 0;
                    resetControl();
                    status = "Zero established";
                } else status = "Zero refused: configure gearing, limits and directions; let shaft stop";
            }
            if (mode == Mode.OFF) { writePower(0); return; }
            if (!controlConfigurationValid()) { fault("Invalid power/PID/feedforward/watchdog settings"); return; }
            double command;
            if (mode == Mode.MANUAL) {
                if (!Double.isFinite(manualRequest) || Math.abs(manualRequest) > 1) {
                    fault("Invalid manual request"); return;
                }
                unclampedPower = manualRequest * manualPower;
                outputLimited = Math.abs(unclampedPower) > maximumPower;
                command = clamp(unclampedPower, -maximumPower, maximumPower);
                status = referenced ? "Manual (travel limits active if enabled)" : "RAW JOG: no turret travel limits before zero";
            } else {
                if (!referenced || !positionConfigurationValid()) {
                    fault("PID needs verified directions, configured gearing/limits and a new zero"); return;
                }
                if (!Double.isFinite(targetDegrees) || (limitTravel && (targetDegrees < minDegrees || targetDegrees > maxDegrees))) {
                    fault("Target outside configured travel"); return;
                }
                command = positionPower();
                if (mode == Mode.FAULT) return;
            }
            if (referenced && limitTravel) {
                double angle = getPositionDegrees();
                if ((angle <= minDegrees && command < 0) || (angle >= maxDegrees && command > 0)) {
                    command = 0; integral = 0;
                    outputLimited = true;
                    status = "Travel limit: only movement back into range allowed";
                }
            }
            if (!Double.isFinite(command)) { fault("Nonfinite controller output"); return; }
            if (!checkMotion(command)) return;
            writePower(command * servoSign);
        } catch (RuntimeException failure) {
            // Stop even if a sensor read or hardware call fails; preserve the original exception.
            try { fault("Hardware exception; reinitialize after checking connections"); }
            catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    private void sample() {
        double[] current = { encoderMinVolts, encoderMaxVolts, encoderSign, servoSign,
                servoTurnsPerTurretTurn, maximumServoDegreesPerSecond, maximumSampleGapMs,
                directionsVerified ? 1 : 0, limitTravel ? 1 : 0, minDegrees, maxDegrees };
        if (!Arrays.equals(current, calibration)) {
            // Even zero output at a travel limit is an armed motion request. A calibration edit
            // must not turn that held command into an unreferenced, unrestricted raw jog.
            boolean armed = calibration != null && (mode == Mode.MANUAL || mode == Mode.POSITION || output != 0);
            calibration = current;
            referenced = false;
            tracker = new ContinuousAngleTracker(maximumServoDegreesPerSecond, maximumSampleGapMs / 1000);
            resetControl();
            if (armed) fault("Calibration changed while motion requested; release, clear fault and re-zero");
        }
        sensorValid = false;
        volts = encoder.getVoltage();
        // A blocking hardware read must count toward the tracking gap before another command.
        long now = clock.getAsLong();
        sampleTime = now;
        wrappedDegrees = Double.NaN;
        if (!Double.isFinite(encoderMinVolts) || !Double.isFinite(encoderMaxVolts)
                || encoderMinVolts < 0 || encoderMaxVolts <= encoderMinVolts
                || !Double.isFinite(volts) || volts < encoderMinVolts || volts > encoderMaxVolts
                || (encoderSign != 1 && encoderSign != -1) || (servoSign != 1 && servoSign != -1)) {
            fault("Invalid encoder voltage, scale or direction"); return;
        }
        wrappedDegrees = ((volts - encoderMinVolts) / (encoderMaxVolts - encoderMinVolts) * 360) % 360;
        if (!Double.isFinite(maximumServoDegreesPerSecond) || maximumServoDegreesPerSecond <= 0
                || !Double.isFinite(maximumSampleGapMs) || maximumSampleGapMs <= 0 || maximumSampleGapMs > 250) {
            referenced = false;
            status = "Set model's maximumServoDegreesPerSecond and sample gap (0..250 ms)";
            if (mode == Mode.MANUAL || mode == Mode.POSITION) fault(status);
            return;
        }
        if (!tracker.update(wrappedDegrees, now)) {
            fault("Encoder tracking lost: " + tracker.getStatus() + "; clear fault and re-zero"); return;
        }
        sensorValid = true;
    }

    private double positionPower() {
        double velocity = getVelocityDegreesPerSecond();
        double dt = controlTime == Long.MIN_VALUE ? 0 : (sampleTime - controlTime) * 1e-9;
        controlTime = sampleTime;
        double[] currentGains = { kP, kI, kD, kS, kV, kA, integralLimit };
        if (!Arrays.equals(gains, currentGains)) { integral = 0; gains = currentGains; }
        if (!updateProfile()) return 0;
        if (moveStarted == Long.MIN_VALUE) moveStarted = sampleTime;
        feedbackPower = feedforwardPower = unclampedPower = 0;
        outputLimited = false;
        double error = desiredPosition - getPositionDegrees();
        double allowedMoveMs = profileDuration * 1000 + moveTimeoutMs;
        // Passing the goal during a running profile is not completion. Continue its braking phase.
        if (profileFinished && Math.abs(getErrorDegrees()) <= toleranceDegrees
                && Math.abs(velocity) <= settledVelocityDegreesPerSecond) {
            integral = 0;
            if (settledSince == Long.MIN_VALUE) settledSince = sampleTime;
            if (isAtTarget()) moveStarted = sampleTime;
            else if ((sampleTime - moveStarted) * 1e-6 >= allowedMoveMs) {
                fault("Position failed to settle before timeout"); return 0;
            }
            status = isAtTarget() ? "At target" : "Settling";
            return 0;
        }
        settledSince = Long.MIN_VALUE;
        if ((sampleTime - moveStarted) * 1e-6 >= allowedMoveMs) { fault("Position move timed out"); return 0; }
        double motionSign = Math.signum(desiredVelocity);
        if (motionSign == 0) motionSign = Math.signum(desiredAcceleration);
        // After the profile ends, friction assist may help correct residual position error.
        if (motionSign == 0 && Math.abs(error) > toleranceDegrees) motionSign = Math.signum(error);
        feedforwardPower = kS * motionSign + kV * desiredVelocity + kA * desiredAcceleration;
        double candidate = clamp(integral + error * dt, -integralLimit, integralLimit);
        double proportionalAndDerivative = kP * error + kD * (desiredVelocity - velocity);
        // Feedforward uses the same output budget; include it when deciding integral windup.
        double raw = proportionalAndDerivative + kI * candidate + feedforwardPower;
        if (Math.abs(raw) <= maximumPower || Math.signum(error) != Math.signum(raw)) integral = candidate;
        feedbackPower = proportionalAndDerivative + kI * integral;
        unclampedPower = feedbackPower + feedforwardPower;
        if (!Double.isFinite(unclampedPower)) { fault("Nonfinite PID/feedforward output"); return 0; }
        outputLimited = Math.abs(unclampedPower) > maximumPower;
        status = profileFinished ? "Correcting final position" : "Following motion profile";
        return clamp(unclampedPower, -maximumPower, maximumPower);
    }

    private boolean updateProfile() {
        if (!profileEnabled) {
            if (profile != null) { integral = 0; moveStarted = settledSince = Long.MIN_VALUE; }
            profile = null; profileDuration = 0;
            desiredPosition = targetDegrees; desiredVelocity = desiredAcceleration = 0;
            profileFinished = true;
            return true;
        }
        if (!positive(maxProfileVelocityDegreesPerSecond) || !positive(maxProfileAccelerationDegreesPerSecondSquared)
                || maxProfileVelocityDegreesPerSecond > maximumServoDegreesPerSecond / servoTurnsPerTurretTurn) {
            fault("Invalid profile constraints or profile speed exceeds configured shaft speed / gearing"); return false;
        }
        boolean replan = profile == null || profileGoal != targetDegrees
                || profileMaxVelocity != maxProfileVelocityDegreesPerSecond
                || profileMaxAcceleration != maxProfileAccelerationDegreesPerSecondSquared;
        if (replan) {
            double startPosition = getPositionDegrees(), startVelocity = getVelocityDegreesPerSecond();
            if (profile != null) {
                TurretMotionProfile.State previous = profile.sample((sampleTime - profileStarted) * 1e-9);
                startPosition = previous.position; startVelocity = previous.velocity;
            }
            try {
                profile = new TurretMotionProfile(startPosition, startVelocity, targetDegrees,
                        maxProfileVelocityDegreesPerSecond, maxProfileAccelerationDegreesPerSecondSquared);
            } catch (IllegalArgumentException invalid) {
                fault("Cannot plan turret motion: " + invalid.getMessage()); return false;
            }
            // Reversing a moving turret can require braking past the new goal. Refuse any plan
            // whose reference would leave the measured travel range, even if its goal is in range.
            if (limitTravel && (profile.minPosition() < minDegrees || profile.maxPosition() > maxDegrees)) {
                fault("Profile braking path exceeds travel limits; stop and reposition manually"); return false;
            }
            profileGoal = targetDegrees;
            profileMaxVelocity = maxProfileVelocityDegreesPerSecond;
            profileMaxAcceleration = maxProfileAccelerationDegreesPerSecondSquared;
            profileStarted = sampleTime;
            profileDuration = profile.duration();
            integral = 0; moveStarted = sampleTime; settledSince = Long.MIN_VALUE;
        }
        double elapsed = (sampleTime - profileStarted) * 1e-9;
        TurretMotionProfile.State state = profile.sample(elapsed);
        desiredPosition = state.position; desiredVelocity = state.velocity; desiredAcceleration = state.acceleration;
        profileFinished = elapsed >= profileDuration;
        return true;
    }

    private boolean checkMotion(double command) {
        if (Math.abs(command) < noMotionPowerThreshold) { motionSince = Long.MIN_VALUE; return true; }
        double direction = Math.signum(command);
        // Acceleration feedforward can brake with reverse power while the turret still moves
        // correctly along the profile. Require progress along that planned motion in this case.
        if (mode == Mode.POSITION && !profileFinished && command * desiredVelocity < 0
                && getVelocityDegreesPerSecond() * desiredVelocity > 0) direction = Math.signum(desiredVelocity);
        double shaft = tracker.getDegrees() * encoderSign;
        if (motionSince == Long.MIN_VALUE || direction != motionDirection
                || (shaft - motionAnchor) * direction >= noMotionMinimumServoDegrees) {
            motionSince = sampleTime; motionAnchor = shaft; motionDirection = direction;
        }
        if ((sampleTime - motionSince) * 1e-6 >= noMotionTimeoutMs) {
            fault("No encoder progress in commanded direction; check wiring, signs or obstruction"); return false;
        }
        return true;
    }

    private boolean positionConfigurationValid() {
        return Double.isFinite(servoTurnsPerTurretTurn) && servoTurnsPerTurretTurn > 0
                && directionsVerified && Double.isFinite(settledVelocityDegreesPerSecond) && settledVelocityDegreesPerSecond > 0
                && (!limitTravel || (Double.isFinite(minDegrees) && Double.isFinite(maxDegrees)
                && minDegrees < maxDegrees && minDegrees <= 0 && maxDegrees >= 0));
    }

    private boolean controlConfigurationValid() {
        return nonnegative(kP) && nonnegative(kI) && nonnegative(kD) && nonnegative(integralLimit)
                && nonnegative(kS) && kS <= 1 && nonnegative(kV) && nonnegative(kA)
                && positive(maximumPower) && maximumPower <= 1 && positive(manualPower) && manualPower <= 0.2
                && positive(toleranceDegrees) && positive(settledVelocityDegreesPerSecond)
                && positive(settleTimeMs) && positive(moveTimeoutMs) && moveTimeoutMs > settleTimeMs
                && positive(noMotionTimeoutMs) && positive(noMotionMinimumServoDegrees)
                && positive(noMotionPowerThreshold) && noMotionPowerThreshold <= Math.min(manualPower, maximumPower);
    }

    public void stop() {
        if (closed) return;
        closed = true; enabled = false; referenced = false; zeroRequested = false;
        mode = Mode.OFF; resetControl(); status = "Stopped";
        writePower(0);
    }

    private void fault(String reason) {
        referenced = false; zeroRequested = false; mode = Mode.FAULT;
        status = reason; resetControl(); writePower(0);
    }

    private void resetControl() {
        integral = 0; controlTime = moveStarted = settledSince = motionSince = Long.MIN_VALUE;
        profile = null; profileGoal = Double.NaN; profileDuration = 0;
        desiredPosition = desiredVelocity = desiredAcceleration = Double.NaN;
        feedbackPower = feedforwardPower = unclampedPower = 0;
        profileFinished = outputLimited = false;
    }

    private void writePower(double power) { output = power; servo.setPower(power); }
    private static boolean positive(double n) { return Double.isFinite(n) && n > 0; }
    private static boolean nonnegative(double n) { return Double.isFinite(n) && n >= 0; }
    private static double clamp(double n, double low, double high) { return Math.max(low, Math.min(high, n)); }
    public Mode getMode() { return mode; }
    public String getStatus() { return status; }
    public double getVoltage() { return volts; }
    public double getWrappedDegrees() { return wrappedDegrees; }
    public double getOutput() { return output; }
    public double getShaftDegrees() { return sensorValid ? tracker.getDegrees() * encoderSign : Double.NaN; }
    public boolean hasReference() { return referenced; }
    public double getPositionDegrees() { return referenced ? (tracker.getDegrees() - zeroShaftDegrees) * encoderSign / servoTurnsPerTurretTurn : Double.NaN; }
    public double getVelocityDegreesPerSecond() { return referenced ? tracker.getVelocityDegreesPerSecond() * encoderSign / servoTurnsPerTurretTurn : Double.NaN; }
    public double getErrorDegrees() { return targetDegrees - getPositionDegrees(); }
    public double getProfilePositionDegrees() { return desiredPosition; }
    public double getProfileVelocityDegreesPerSecond() { return desiredVelocity; }
    public double getProfileAccelerationDegreesPerSecondSquared() { return desiredAcceleration; }
    public double getProfileDurationSeconds() { return profileDuration; }
    public double getFeedbackPower() { return feedbackPower; }
    public double getFeedforwardPower() { return feedforwardPower; }
    public double getUnclampedPower() { return unclampedPower; }
    public boolean isOutputLimited() { return outputLimited; }
    public boolean isProfileFinished() { return profileFinished; }
    public boolean isAtTarget() {
        return mode == Mode.POSITION && referenced && sensorValid && profileFinished && settledSince != Long.MIN_VALUE
                && clock.getAsLong() >= sampleTime
                && (clock.getAsLong() - sampleTime) * 1e-6 <= maximumSampleGapMs
                && (sampleTime - settledSince) * 1e-6 >= settleTimeMs;
    }
}
