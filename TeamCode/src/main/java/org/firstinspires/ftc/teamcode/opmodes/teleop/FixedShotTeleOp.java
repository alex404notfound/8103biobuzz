package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

import java.util.HashSet;

/** Robot-relative driving, manual turret aiming, and fixed shots with a single shooter. */
@Configurable
@TeleOp(name = "Fixed Shot TeleOp", group = "Competition")
public class FixedShotTeleOp extends OpMode {
    public static WedgeSettings wedge = new WedgeSettings();
    public static volatile double intakePower = 1;
    public static volatile double turretMinPosition = 0.40, turretMaxPosition = 0.60;
    public static volatile double turretPositionPerSecond = 0.25;
    public static volatile boolean turretReversed = false;

    public static class WedgeSettings {
        public volatile double stowedPosition = -1, deployedPosition = -1;
    }

    private static final double DEFAULT_SHOT_RPM = 3000;
    private static final double SHOT_RPM_TOLERANCE = 300;

    private DcMotorEx[] brakedMotors = new DcMotorEx[0];
    private Mecanum drive;
    private DcMotorEx intake, shooter;
    private Servo stopper, turret;
    private Wedge wedgeControl;
    private ShooterFlywheel flywheel;
    private boolean running, stopped, intakeOn, shotCancelled;
    private boolean turretReleaseRequired;
    private double turretTarget, lastTurretUpdate;

    @Override public void init() {
        stop();
        stopped = false;
        intakeOn = shotCancelled = false;
        shooter = null;
        stopper = null;
        turret = null;
        brakedMotors = new DcMotorEx[0];
        try {
            intake = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR_NAME);
            DcMotorEx mappedShooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
            Servo mappedStopper = hardwareMap.tryGet(Servo.class, "shotStopper");
            Servo mappedWedge = hardwareMap.tryGet(Servo.class, Constants.WEDGE_SERVO_NAME);
            Servo mappedTurret = hardwareMap.tryGet(Servo.class, Constants.TURRET_SERVO_NAME);
            wedgeControl = new Wedge(mappedWedge, wedge);
            MecanumConfig config = Constants.drivetrainConfig;
            DcMotorEx fl = hardwareMap.get(DcMotorEx.class, config.frontLeftName.get());
            DcMotorEx fr = hardwareMap.get(DcMotorEx.class, config.frontRightName.get());
            DcMotorEx bl = hardwareMap.get(DcMotorEx.class, config.backLeftName.get());
            DcMotorEx br = hardwareMap.get(DcMotorEx.class, config.backRightName.get());
            requireDistinct(fl, fr, bl, br, intake, mappedShooter);
            requireDistinct(mappedWedge, mappedStopper, mappedTurret);
            shooter = mappedShooter;
            stopper = mappedStopper;
            brakedMotors = new DcMotorEx[]{fl, fr, bl, br, intake};
            MotorSafety.stopAndBrake(brakedMotors);
            turret = mappedTurret;
            turretTarget = turret != null ? turret.getPosition() : Double.NaN;
            for (DcMotorEx motor : brakedMotors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setDirection(Constants.INTAKE_MOTOR_DIRECTION);
            flywheel = ShooterFlywheel.forShooter(shooter, hardwareMap, SHOT_RPM_TOLERANCE);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive = Constants.createDrivetrain(hardwareMap);
            drive.stop(true);
            if (stopper != null) stopper.setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
            wedgeControl.stow();
            telemetry = RobotTelemetry.forOpMode(telemetry);
            report(false, false);
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void start() {
        if (stopped) return;
        gamepad1.resetEdgeDetection();
        wedgeControl.buttonWasHeld = gamepad1.left_bumper;
        turretReleaseRequired = deadband(gamepad1.left_trigger) != 0 || deadband(gamepad1.right_trigger) != 0;
        lastTurretUpdate = getRuntime();
        running = true;
    }

    @Override public void loop() {
        if (!running || stopped) return;
        try {
            boolean toggleIntake = gamepad1.circleWasPressed();
            boolean cancel = gamepad1.square;
            if (!gamepad1.cross) shotCancelled = false;
            if (cancel) {
                intakeOn = false;
                shotCancelled = true;
            } else if (stopper != null && toggleIntake) {
                intakeOn = !intakeOn;
            }
            updateTurret(cancel);

            double forward = deadband(-gamepad1.left_stick_y);
            double left = deadband(-gamepad1.left_stick_x);
            double turn = deadband(-gamepad1.right_stick_x);
            if (cancel || (forward == 0 && left == 0 && turn == 0)) drive.stop(true);
            else drive.drive(new DrivePowers(forward, left, turn), true);

            boolean shooting = stopper != null && gamepad1.cross && !shotCancelled;
            if (shooting) {
                double rpm = ShooterFlywheel.SHOOTER.setpointRpm;
                flywheel.updateRpm(rpm == 0 ? DEFAULT_SHOT_RPM : rpm, ShooterFlywheel.SHOOTER);
            } else if (flywheel.getMode() != ShooterFlywheel.Mode.OFF) {
                flywheel.idle();
            }

            boolean feeding = shooting && flywheel.isWithinTargetSpeed();
            if (stopper != null) {
                stopper.setPosition(feeding
                        ? Constants.SHOT_STOPPER_UNBLOCKED_POSITION : Constants.SHOT_STOPPER_BLOCKED_POSITION);
            }
            // During a shot, the flywheel's speed gate takes priority over manual intake.
            boolean intakeRunning = shooting ? feeding : intakeOn;
            applyPower(intake, intakeRunning ? intakePower : 0);
            wedgeControl.update(gamepad1.left_bumper);
            report(feeding, intakeRunning);
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void updateTurret(boolean cancel) {
        if (turret == null) return;
        double now = getRuntime();
        // A stalled loop should not turn one trigger sample into a large position change.
        double elapsed = Math.min(now - lastTurretUpdate, 0.05);
        lastTurretUpdate = now;
        double left = deadband(gamepad1.left_trigger);
        double right = deadband(gamepad1.right_trigger);
        boolean released = left == 0 && right == 0;
        turretReleaseRequired = cancel || (turretReleaseRequired && !released);
        if (turretReleaseRequired || released || (left != 0 && right != 0)) return;

        double min = turretMinPosition, max = turretMaxPosition, rate = turretPositionPerSecond;
        double input = right - left;
        if (!(servoPosition(min) && servoPosition(max) && min < max && rate > 0 && Double.isFinite(rate)
                && Math.abs(input) <= 1 && elapsed > 0)) return;

        double start = servoPosition(turretTarget) ? turretTarget : (min + max) / 2;
        double delta = input * rate * elapsed * (turretReversed ? -1 : 1);
        double target = Math.max(min, Math.min(max, start + delta));
        if (target != turretTarget) {
            turret.setPosition(target);
            turretTarget = target;
        }
    }

    private void report(boolean feeding, boolean intakeRunning) {
        String status = flywheel.getMode() == ShooterFlywheel.Mode.OFF ? "Idle" : flywheel.getStatus();
        if (feeding) status = "Feeding";
        telemetry.addData("Shot status", status);
        telemetry.addData("Shooter setpoint RPM", flywheel.getTargetRpm());
        telemetry.addData("Shooter measured RPM", flywheel.getMeasuredRpm());
        telemetry.addData("Intake/transfer on", intakeRunning);
        telemetry.addLine("GP1 sticks to drive. Square cancels.");
        if (turret != null) {
            telemetry.addLine("Hold L2 or R2 to aim the turret left or right.");
            if (servoPosition(turretTarget)) telemetry.addData("Turret target", "%.3f", turretTarget);
        }
        if (wedgeControl.servo != null) telemetry.addLine("L1 toggles the wedge.");
        if (stopper != null) {
            telemetry.addLine("Tap Circle to toggle the intake/transfer motor.");
            telemetry.addLine("Hold Cross to shoot once the flywheel is within 300 RPM of the target.");
        } else telemetry.addLine("Shot stopper unavailable. Shooting and feeding disabled.");
        telemetry.update();
    }

    @Override public void stop() {
        if (stopped) return;
        running = false;
        stopped = true;
        try { if (shooter != null) shooter.setPower(0); }
        finally {
            try { MotorSafety.stopAndBrake(brakedMotors); }
            finally {
                if (stopper != null) stopper.setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
            }
        }
    }

    private void shutdown(RuntimeException failure) {
        try { stop(); }
        catch (RuntimeException stopFailure) { if (failure != stopFailure) failure.addSuppressed(stopFailure); }
        throw failure;
    }

    private static double deadband(double input) { return Math.abs(input) < 0.08 ? 0 : input; }

    private static void applyPower(DcMotorEx motor, double power) {
        motor.setPower(Math.abs(power) <= 1 ? power : 0);
    }

    private static boolean servoPosition(double value) { return value >= 0 && value <= 1; }

    private static void requireDistinct(HardwareDevice... devices) {
        HashSet<HardwareDevice> mapped = new HashSet<>();
        for (HardwareDevice device : devices) {
            if (device != null && !mapped.add(device))
                throw new IllegalArgumentException("Each drive motor, mechanism motor, and servo needs a distinct device");
        }
    }

    private static final class Wedge {
        final Servo servo;
        final double stowedPosition, deployedPosition;
        final boolean configured;
        boolean deployed, buttonWasHeld;

        Wedge(Servo servo, WedgeSettings settings) {
            this.servo = servo;
            stowedPosition = settings.stowedPosition;
            deployedPosition = settings.deployedPosition;
            configured = servo != null && servoPosition(stowedPosition) && servoPosition(deployedPosition)
                    && stowedPosition != deployedPosition;
        }

        void stow() { if (configured) servo.setPosition(stowedPosition); }

        void update(boolean held) {
            boolean pressed = held && !buttonWasHeld;
            buttonWasHeld = held;
            if (configured && pressed) {
                deployed = !deployed;
                servo.setPosition(deployed ? deployedPosition : stowedPosition);
            }
        }
    }
}
