package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/** Two motors on one linked flywheel + one positional servo, controlled from FTC Dashboard. */
@Config
@TeleOp(name = "Shooter Tuning", group = "Prototyping")
public class ShooterTuning extends OpMode {
    // Hardware names and direction apply at INIT. RPM is measured at the motor shaft.
    public static volatile String leftMotorName = "launcherLeft", rightMotorName = "launcherRight", servoName = "hood";
    public static volatile boolean leftReversed = false, rightReversed = false;
    public static volatile double ticksPerRevolution = 28;
    public static volatile boolean runFlywheel = false;
    public static volatile double targetRpm = 1000, servoPosition = 0.5;
    // Gains produce shared motor power, using RPM and seconds.
    public static volatile double kP = 0, kI = 0, kD = 0, kS = 0, kV = 0.0002, kA = 0;
    public static volatile double maxPower = 1.0, maxAccelerationRpmPerSecond = 3000;
    public static volatile double integralLimit = 1000; // RPM * seconds.

    private final LongSupplier clock;
    private final Supplier<Telemetry> dashboardTelemetry;
    private DcMotorEx leftMotor, rightMotor;
    private Servo servo;
    private List<LynxModule> hubs = Collections.emptyList();
    private boolean active, stopped, running, initialLeftReversed, initialRightReversed;
    private long previousTime;
    private double referenceRpm, previousRpm, previousTarget, integral, power;
    private double feedback, feedforward;
    private double pPower, iPower, dPower, requestedPower;
    private double[] appliedGains;
    private boolean outputLimited;
    private String status = "OFF";

    public ShooterTuning() {
        this(System::nanoTime, () -> FtcDashboard.getInstance().getTelemetry());
    }

    ShooterTuning(LongSupplier clock, Supplier<Telemetry> dashboardTelemetry) {
        this.clock = clock;
        this.dashboardTelemetry = dashboardTelemetry;
    }

    @Override public void init() {
        active = stopped = false;
        runFlywheel = false;
        resetController();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry.get());
        telemetry.setMsTransmissionInterval(50);
        try {
            leftMotor = hardwareMap.get(DcMotorEx.class, leftMotorName);
            leftMotor.setPower(0);
            rightMotor = hardwareMap.get(DcMotorEx.class, rightMotorName);
            rightMotor.setPower(0);
            if (leftMotor == rightMotor) throw new IllegalArgumentException("Configure two distinct shooter motors");
            initialLeftReversed = leftReversed;
            initialRightReversed = rightReversed;
            configureMotor(leftMotor, initialLeftReversed);
            configureMotor(rightMotor, initialRightReversed);
            servo = hardwareMap.get(Servo.class, servoName);
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException failure) {
            stopAfterFailure(failure);
            throw failure;
        }
    }

    private static void configureMotor(DcMotorEx motor, boolean reversed) {
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override public void init_loop() {
        if (stopped) return;
        telemetry.addLine("Dashboard Config: ShooterTuning | launcherLeft + launcherRight + hood");
        telemetry.addLine("START, set targetRpm, then runFlywheel=true. Zero output brakes both motors.");
        telemetry.update();
    }

    @Override public void start() {
        if (stopped) return;
        active = true;
        runFlywheel = false; // Dashboard values survive OpMode runs; require a fresh run command.
        resetController();
        previousTime = clock.getAsLong();
        zeroMotors();
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            // The left encoder measures the shared flywheel; no right encoder is required.
            double rpm = leftMotor.getVelocity() * 60 / ticksPerRevolution;
            long now = clock.getAsLong();
            double dt = (now - previousTime) * 1e-9;
            previousTime = now;
            double target = targetRpm, position = servoPosition;

            if (!validSettings(target, position) || !Double.isFinite(rpm)) {
                disarm("Invalid gains, limits, servo position or encoder reading");
            } else if (leftReversed != initialLeftReversed || rightReversed != initialRightReversed) {
                disarm("Direction changed: STOP and INIT to apply");
            } else {
                // The servo is independent of the flywheel run toggle.
                servo.setPosition(clamp(position, 0, 1));
                if (!runFlywheel || target == 0) {
                    resetController();
                    status = "OFF (BRAKE)";
                } else if (dt <= 0 || dt > 0.25) {
                    disarm("Loop timing gap: set runFlywheel=true to retry");
                } else {
                    updateController(target, rpm, dt);
                }
            }
            leftMotor.setPower(power);
            rightMotor.setPower(power);
            telemetry.addData("shooter.status", status);
            telemetry.addData("shooter.targetRpm", target);
            telemetry.addData("shooter.referenceRpm", referenceRpm);
            telemetry.addData("shooter.measuredRpm", rpm);
            telemetry.addData("shooter.errorRpm", referenceRpm - rpm);
            telemetry.addData("shooter.feedforwardPower", feedforward);
            telemetry.addData("shooter.feedbackPower", feedback);
            telemetry.addData("shooter.pPower", pPower);
            telemetry.addData("shooter.iPower", iPower);
            telemetry.addData("shooter.dPower", dPower);
            telemetry.addData("shooter.requestedPower", requestedPower);
            telemetry.addData("shooter.outputLimited", outputLimited);
            telemetry.addData("shooter.leftPower", power);
            telemetry.addData("shooter.rightPower", power);
            telemetry.addData("shooter.servoPosition", servo.getPosition());
            telemetry.addLine("Dashboard: ShooterTuning | runFlywheel=false stops both motors | feedback: left encoder");
            telemetry.update();
        } catch (RuntimeException failure) {
            stopAfterFailure(failure);
            throw failure;
        }
    }

    private void updateController(double target, double rpm, double dt) {
        if (!running) {
            referenceRpm = Math.max(0, rpm);
            previousRpm = rpm;
            running = true;
        }
        double p = kP, i = kI, d = kD, s = kS, v = kV, a = kA, integralCap = integralLimit;
        double[] gains = {p, i, d, s, v, a, integralCap};
        // Old integral correction belongs to the old gains, not a fresh tuning experiment.
        if (target != previousTarget || !Arrays.equals(appliedGains, gains)) integral = 0;
        appliedGains = gains;
        previousTarget = target;

        double oldReference = referenceRpm;
        double maxChange = maxAccelerationRpmPerSecond * dt;
        // Ramp up, but apply lower targets immediately so slowing down is not rate-limited.
        referenceRpm = Math.min(target, referenceRpm + maxChange);
        double acceleration = (referenceRpm - oldReference) / dt;
        double error = referenceRpm - rpm;
        double measuredAcceleration = (rpm - previousRpm) / dt;
        previousRpm = rpm;
        double limit = maxPower;
        double candidateIntegral = i == 0 ? 0 : clamp(integral + error * dt, -integralCap, integralCap);
        feedforward = (referenceRpm > 0 ? s : 0) + v * referenceRpm + a * acceleration;
        pPower = p * error;
        dPower = -d * measuredAcceleration; // Derivative on measurement avoids target-step kick.
        double base = feedforward + pPower + dPower;
        double requested = base + i * candidateIntegral;
        // Do not accumulate error when it would push a saturated output further into its limit.
        if (!((requested > limit && error > 0) || (requested < 0 && error < 0))) {
            integral = candidateIntegral;
        }
        iPower = i * integral;
        feedback = pPower + iPower + dPower;
        requestedPower = feedforward + feedback;
        if (!Double.isFinite(requestedPower)) {
            disarm("Nonfinite controller output");
            return;
        }
        outputLimited = requestedPower < 0 || requestedPower > limit;
        power = clamp(requestedPower, 0, limit); // Zero engages BRAKE; never command reverse.
        status = "RUNNING RPM";
    }

    private static boolean validSettings(double target, double position) {
        if (!Double.isFinite(position) || ticksPerRevolution <= 0 || maxAccelerationRpmPerSecond <= 0
                || maxPower > 1 || target > 6000) return false;
        for (double value : new double[]{target, ticksPerRevolution, kP, kI, kD, kS, kV, kA,
                maxPower, maxAccelerationRpmPerSecond, integralLimit}) {
            if (!Double.isFinite(value) || value < 0) return false;
        }
        return true;
    }

    private void resetController() {
        running = false;
        referenceRpm = previousRpm = previousTarget = integral = power = feedback = feedforward = 0;
        pPower = iPower = dPower = requestedPower = 0;
        appliedGains = null;
        outputLimited = false;
    }

    private void disarm(String reason) {
        runFlywheel = false;
        resetController();
        status = reason;
    }

    private void stopAfterFailure(RuntimeException failure) {
        try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        active = runFlywheel = false;
        resetController();
        zeroMotors();
    }

    private void zeroMotors() {
        try { if (leftMotor != null) leftMotor.setPower(0); }
        finally { if (rightMotor != null) rightMotor.setPower(0); }
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
