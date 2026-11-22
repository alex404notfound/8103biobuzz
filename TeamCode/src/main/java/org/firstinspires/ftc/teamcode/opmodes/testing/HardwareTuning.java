package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.opmodes.teleop.FixedShotTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

/** Live individual controls for the motors and servos in the two-hub configuration. */
@Configurable
@TeleOp(name = "Hardware Tuning", group = "Tuning")
public class HardwareTuning extends OpMode {
    public static volatile double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    public static volatile double intakePower, shooterPower, shooter2Power;
    public static volatile double wedgePosition = -1, shotStopperPosition = -1, turretPosition = -1;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake, shooter, shooter2;
    private Servo wedge, shotStopper, turret;
    private DcMotorEx[] motors = new DcMotorEx[0];

    @Override public void init() {
        zeroPowerSettings();
        wedgePosition = shotStopperPosition = turretPosition = -1;
        try {
            MecanumConfig drive = Constants.drivetrainConfig;
            frontLeft = hardwareMap.get(DcMotorEx.class, drive.frontLeftName.get());
            frontRight = hardwareMap.get(DcMotorEx.class, drive.frontRightName.get());
            backLeft = hardwareMap.get(DcMotorEx.class, drive.backLeftName.get());
            backRight = hardwareMap.get(DcMotorEx.class, drive.backRightName.get());
            intake = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR_NAME);
            shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
            shooter2 = hardwareMap.tryGet(DcMotorEx.class, Constants.SECOND_SHOOTER_MOTOR_NAME);
            motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight, intake, shooter, shooter2};
            MotorSafety.stopAndBrake(motors);
            wedge = hardwareMap.tryGet(Servo.class, Constants.WEDGE_SERVO_NAME);
            shotStopper = hardwareMap.tryGet(Servo.class, "shotStopper");
            turret = hardwareMap.tryGet(Servo.class, Constants.TURRET_SERVO_NAME);
            for (DcMotorEx motor : motors) {
                if (motor != null) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            frontLeft.setDirection(drive.frontLeftDirection.get());
            frontRight.setDirection(drive.frontRightDirection.get());
            backLeft.setDirection(drive.backLeftDirection.get());
            backRight.setDirection(drive.backRightDirection.get());
            intake.setDirection(Constants.INTAKE_MOTOR_DIRECTION);
            shooter.setDirection(ShooterFlywheel.SHOOTER.reversed
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            if (shooter2 != null) {
                shooter2.setDirection(FlywheelTuning.speed2.reversed
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }
            telemetry = RobotTelemetry.forOpMode(telemetry);
            telemetry.addLine("After START, open Panels > Configurables > HardwareTuning.");
            telemetry.addLine("Motor power: -1 to 1. Servo position: 0 to 1.");
            telemetry.addLine("Turret position uses the FixedShotTeleOp travel limits.");
            reportConnection("frontLeftPower", drive.frontLeftName.get(), frontLeft);
            reportConnection("frontRightPower", drive.frontRightName.get(), frontRight);
            reportConnection("backLeftPower", drive.backLeftName.get(), backLeft);
            reportConnection("backRightPower", drive.backRightName.get(), backRight);
            reportConnection("intakePower", Constants.INTAKE_MOTOR_NAME, intake);
            reportConnection("shooterPower", Constants.SHOOTER_MOTOR_NAME, shooter);
            reportConnection("shooter2Power", Constants.SECOND_SHOOTER_MOTOR_NAME, shooter2);
            reportConnection("wedgePosition", Constants.WEDGE_SERVO_NAME, wedge);
            reportConnection("shotStopperPosition", "shotStopper", shotStopper);
            reportConnection("turretPosition", Constants.TURRET_SERVO_NAME, turret);
            telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void loop() {
        try {
            applyPower(frontLeft, frontLeftPower);
            applyPower(frontRight, frontRightPower);
            applyPower(backLeft, backLeftPower);
            applyPower(backRight, backRightPower);
            applyPower(intake, intakePower);
            applyPower(shooter, shooterPower);
            if (shooter2 != null) applyPower(shooter2, shooter2Power);
            applyPosition(wedge, wedgePosition);
            applyPosition(shotStopper, shotStopperPosition);
            double position = turretPosition;
            double min = FixedShotTeleOp.turretMinPosition, max = FixedShotTeleOp.turretMaxPosition;
            if (min >= 0 && min < max && max <= 1 && position >= min && position <= max) {
                applyPosition(turret, position);
            }
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void stop() {
        zeroPowerSettings();
        MotorSafety.stopAndBrake(motors);
    }

    private static void zeroPowerSettings() {
        frontLeftPower = frontRightPower = backLeftPower = backRightPower = 0;
        intakePower = shooterPower = shooter2Power = 0;
    }

    private void reportConnection(String control, String name, HardwareDevice device) {
        telemetry.addData(control, name + " — " + (device == null ? "unavailable" : device.getConnectionInfo()));
    }

    private static void applyPower(DcMotorEx motor, double power) {
        motor.setPower(Math.abs(power) <= 1 ? power : 0);
    }

    private static void applyPosition(Servo servo, double position) {
        if (servo != null && position >= 0 && position <= 1) servo.setPosition(position);
    }

    private void shutdown(RuntimeException failure) {
        try { stop(); }
        catch (RuntimeException stopFailure) { if (failure != stopFailure) failure.addSuppressed(stopFailure); }
        throw failure;
    }
}
