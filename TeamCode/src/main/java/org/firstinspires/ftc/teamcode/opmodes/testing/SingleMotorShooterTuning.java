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

import java.util.Collections;
import java.util.List;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/** One flywheel motor + one positional servo, controlled entirely from FTC Dashboard. */
@Config
@TeleOp(name = "Single Motor Shooter Tuning", group = "Prototyping")
public class SingleMotorShooterTuning extends OpMode {
    // Hardware names and direction apply at INIT. RPM is measured at the motor shaft.
    public static volatile String motorName = "launcherLeft", servoName = "hood";
    public static volatile boolean motorReversed = false;
    public static volatile double ticksPerRevolution = 28;
    public static volatile boolean runFlywheel = false;
    public static volatile double targetRpm = 1000, servoPosition = 0.5;
    // Gains produce motor power, using RPM and seconds (not the two-motor tuner's volts).
    public static volatile double kP = 0.0002, kI = 0, kD = 0, kV = 1.0 / 6000, kA = 0;
    public static volatile double maxPower = 0.5, maxAccelerationRpmPerSecond = 3000;
    public static volatile double integralLimit = 1000; // RPM * seconds.

    private final LongSupplier clock;
    private final Supplier<Telemetry> dashboardTelemetry;
    private DcMotorEx motor;
    private Servo servo;
    private List<LynxModule> hubs = Collections.emptyList();
    private boolean active, stopped, running, initialMotorReversed;
    private long previousTime;
    private double referenceRpm, previousRpm, previousTarget, integral, power;
    private double feedback, feedforward;
    private String status = "OFF";

    public SingleMotorShooterTuning() {
        this(System::nanoTime, () -> FtcDashboard.getInstance().getTelemetry());
    }

    SingleMotorShooterTuning(LongSupplier clock, Supplier<Telemetry> dashboardTelemetry) {
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
            motor = hardwareMap.get(DcMotorEx.class, motorName);
            motor.setPower(0);
            initialMotorReversed = motorReversed;
            motor.setDirection(initialMotorReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            servo = hardwareMap.get(Servo.class, servoName);
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (RuntimeException failure) {
            stopAfterFailure(failure);
            throw failure;
        }
    }

    @Override public void init_loop() {
        if (stopped) return;
        telemetry.addLine("Dashboard Config: SingleMotorShooterTuning");
        telemetry.addLine("START, then set runFlywheel=true to hold targetRpm. Servo position applies after START.");
        telemetry.update();
    }

    @Override public void start() {
        if (stopped) return;
        active = true;
        runFlywheel = false; // Dashboard values survive OpMode runs; require a fresh run command.
        resetController();
        previousTime = clock.getAsLong();
        motor.setPower(0);
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            double rpm = motor.getVelocity() * 60 / ticksPerRevolution;
            long now = clock.getAsLong();
            double dt = (now - previousTime) * 1e-9;
            previousTime = now;
            double target = targetRpm, position = servoPosition;

            if (!validSettings(target, position) || !Double.isFinite(rpm)) {
                disarm("Invalid gains, limits, servo position or encoder reading");
            } else if (motorReversed != initialMotorReversed) {
                disarm("Direction changed: STOP and INIT to apply");
            } else {
                // The servo is independent of the flywheel run toggle.
                servo.setPosition(clamp(position, 0, 1));
                if (!runFlywheel || target == 0) {
                    resetController();
                    status = "OFF (coasting)";
                } else if (dt <= 0 || dt > 0.25) {
                    disarm("Loop timing gap: set runFlywheel=true to retry");
                } else {
                    updateController(target, rpm, dt);
                }
            }
            motor.setPower(power);
            telemetry.addData("shooter.status", status);
            telemetry.addData("shooter.targetRpm", target);
            telemetry.addData("shooter.referenceRpm", referenceRpm);
            telemetry.addData("shooter.measuredRpm", rpm);
            telemetry.addData("shooter.errorRpm", referenceRpm - rpm);
            telemetry.addData("shooter.feedforwardPower", feedforward);
            telemetry.addData("shooter.feedbackPower", feedback);
            telemetry.addData("shooter.motorPower", power);
            telemetry.addData("shooter.servoPosition", servo.getPosition());
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
        if (target != previousTarget) integral = 0;
        previousTarget = target;

        double oldReference = referenceRpm;
        double maxChange = maxAccelerationRpmPerSecond * dt;
        referenceRpm += clamp(target - referenceRpm, -maxChange, maxChange);
        double acceleration = (referenceRpm - oldReference) / dt;
        double error = referenceRpm - rpm;
        double derivative = -(rpm - previousRpm) / dt; // Avoid derivative kick on RPM edits.
        previousRpm = rpm;
        double p = kP, i = kI, d = kD, limit = maxPower;
        double candidateIntegral = i == 0 ? 0 : clamp(integral + error * dt, -integralLimit, integralLimit);
        feedforward = kV * referenceRpm + kA * acceleration;
        double base = feedforward + p * error + d * derivative;
        double requested = base + i * candidateIntegral;
        // Do not accumulate error when it would push a saturated output further into its limit.
        if (!((requested > limit && error > 0) || (requested < 0 && error < 0))) {
            integral = candidateIntegral;
        }
        feedback = p * error + i * integral + d * derivative;
        requested = feedforward + feedback;
        if (!Double.isFinite(requested)) {
            disarm("Nonfinite controller output");
            return;
        }
        power = clamp(requested, 0, limit); // Reduce power to coast; never actively reverse.
        status = "RUNNING";
    }

    private static boolean validSettings(double target, double position) {
        if (!Double.isFinite(position) || ticksPerRevolution <= 0 || maxAccelerationRpmPerSecond <= 0
                || maxPower > 1) return false;
        for (double value : new double[]{target, ticksPerRevolution, kP, kI, kD, kV, kA,
                maxPower, maxAccelerationRpmPerSecond, integralLimit}) {
            if (!Double.isFinite(value) || value < 0) return false;
        }
        return true;
    }

    private void resetController() {
        running = false;
        referenceRpm = previousRpm = previousTarget = integral = power = feedback = feedforward = 0;
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
        stopped = true;
        active = runFlywheel = false;
        resetController();
        if (motor != null) motor.setPower(0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
