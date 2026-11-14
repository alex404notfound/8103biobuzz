package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

import java.util.Arrays;
import java.util.HashSet;

/** Robot-relative driving, one wedge, and fixed shots with a single shooter. */
@Configurable
@TeleOp(name = "Fixed Shot TeleOp", group = "Competition")
public class FixedShotTeleOp extends OpMode {
    public static WedgeSettings wedge = new WedgeSettings();
    public static volatile double intakePower = 1, transferPower = 1;

    public static class WedgeSettings {
        public volatile double stowedPosition = -1, deployedPosition = -1;
    }

    private static final double DEFAULT_SHOT_RPM = 3000;
    private static final double SHOT_RPM_TOLERANCE = 300;

    private DcMotorEx[] brakedMotors = new DcMotorEx[0];
    private Mecanum drive;
    private DcMotorEx intake, transfer, shooter;
    private Servo stopper;
    private Wedge wedgeControl;
    private ShooterFlywheel flywheel;
    private boolean running, stopped, intakeOn, transferOn, shotCancelled;

    @Override public void init() {
        stop();
        stopped = false;
        intakeOn = transferOn = shotCancelled = false;
        shooter = null;
        stopper = null;
        brakedMotors = new DcMotorEx[0];
        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            transfer = hardwareMap.get(DcMotorEx.class, Constants.TRANSFER_MOTOR_NAME);
            DcMotorEx mappedShooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
            Servo mappedStopper = hardwareMap.get(Servo.class, "shotStopper");
            wedgeControl = new Wedge(hardwareMap.get(Servo.class, Constants.WEDGE_SERVO_NAME), wedge);
            MecanumConfig config = Constants.drivetrainConfig;
            DcMotorEx fl = hardwareMap.get(DcMotorEx.class, config.frontLeftName.get());
            DcMotorEx fr = hardwareMap.get(DcMotorEx.class, config.frontRightName.get());
            DcMotorEx bl = hardwareMap.get(DcMotorEx.class, config.backLeftName.get());
            DcMotorEx br = hardwareMap.get(DcMotorEx.class, config.backRightName.get());
            requireDistinct(fl, fr, bl, br, intake, transfer, mappedShooter);
            requireDistinct(wedgeControl.servo, mappedStopper);
            shooter = mappedShooter;
            stopper = mappedStopper;
            brakedMotors = new DcMotorEx[]{fl, fr, bl, br, intake, transfer};
            MotorSafety.stopAndBrake(brakedMotors);
            for (DcMotorEx motor : brakedMotors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel = ShooterFlywheel.forShooter(shooter, hardwareMap, SHOT_RPM_TOLERANCE);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive = Constants.createDrivetrain(hardwareMap);
            drive.stop(true);
            stopper.setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
            wedgeControl.stow();
            telemetry = RobotTelemetry.forOpMode(telemetry);
            report(false);
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void start() {
        if (stopped) return;
        gamepad1.resetEdgeDetection();
        wedgeControl.buttonWasHeld = gamepad1.left_bumper;
        running = true;
    }

    @Override public void loop() {
        if (!running || stopped) return;
        try {
            boolean toggleIntake = gamepad1.bWasPressed();
            boolean toggleTransfer = gamepad1.yWasPressed();
            boolean cancel = gamepad1.x;
            if (!gamepad1.a) shotCancelled = false;
            if (cancel) {
                intakeOn = transferOn = false;
                shotCancelled = true;
            } else {
                if (toggleIntake) intakeOn = !intakeOn;
                if (toggleTransfer) transferOn = !transferOn;
            }

            double forward = deadband(-gamepad1.left_stick_y);
            double left = deadband(-gamepad1.left_stick_x);
            double turn = deadband(-gamepad1.right_stick_x);
            if (cancel || (forward == 0 && left == 0 && turn == 0)) drive.stop(true);
            else drive.drive(new DrivePowers(forward, left, turn), true);

            boolean shooting = gamepad1.a && !shotCancelled;
            if (shooting) {
                double rpm = ShooterFlywheel.SHOOTER.setpointRpm;
                flywheel.updateRpm(rpm == 0 ? DEFAULT_SHOT_RPM : rpm, ShooterFlywheel.SHOOTER);
            } else if (flywheel.getMode() != ShooterFlywheel.Mode.OFF) {
                flywheel.idle();
            }

            boolean feeding = shooting && flywheel.isWithinTargetSpeed();
            stopper.setPosition(feeding
                    ? Constants.SHOT_STOPPER_UNBLOCKED_POSITION : Constants.SHOT_STOPPER_BLOCKED_POSITION);
            applyPower(intake, intakeOn ? intakePower : 0);
            applyPower(transfer, transferOn || feeding ? transferPower : 0);
            wedgeControl.update(gamepad1.left_bumper);
            report(feeding);
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void report(boolean feeding) {
        String status = flywheel.getMode() == ShooterFlywheel.Mode.OFF ? "Idle" : flywheel.getStatus();
        if (feeding) status = "Feeding";
        telemetry.addData("Shot status", status);
        telemetry.addData("Shooter setpoint RPM", flywheel.getTargetRpm());
        telemetry.addData("Shooter measured RPM", flywheel.getMeasuredRpm());
        telemetry.addData("Intake on", intakeOn);
        telemetry.addData("Transfer on", transferOn || feeding);
        telemetry.addLine("GP1 sticks to drive, left bumper toggles the wedge.");
        telemetry.addLine("Tap B to toggle intake and Y to toggle transfer.");
        telemetry.addLine("Hold A to shoot within 300 RPM of the target. X cancels.");
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
        if (new HashSet<>(Arrays.asList(devices)).size() != devices.length)
            throw new IllegalArgumentException("Each drive motor, mechanism motor, and servo needs a distinct device");
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
            configured = servoPosition(stowedPosition) && servoPosition(deployedPosition)
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
