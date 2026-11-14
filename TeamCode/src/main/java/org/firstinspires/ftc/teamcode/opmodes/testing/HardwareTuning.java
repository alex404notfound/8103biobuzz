package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

/** Live individual controls for the motors and servos in the two-hub configuration. */
@Configurable
@TeleOp(name = "Hardware Tuning", group = "Tuning")
public class HardwareTuning extends OpMode {
    public static volatile double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    public static volatile double intakePower, transferPower, shooterPower;
    public static volatile double wedgePosition = -1, shotStopperPosition = -1;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intake, transfer, shooter;
    private Servo wedge, shotStopper;
    private DcMotorEx[] motors = new DcMotorEx[0];

    @Override public void init() {
        zeroPowerSettings();
        wedgePosition = shotStopperPosition = -1;
        try {
            MecanumConfig drive = Constants.drivetrainConfig;
            frontLeft = hardwareMap.get(DcMotorEx.class, drive.frontLeftName.get());
            frontRight = hardwareMap.get(DcMotorEx.class, drive.frontRightName.get());
            backLeft = hardwareMap.get(DcMotorEx.class, drive.backLeftName.get());
            backRight = hardwareMap.get(DcMotorEx.class, drive.backRightName.get());
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            transfer = hardwareMap.get(DcMotorEx.class, Constants.TRANSFER_MOTOR_NAME);
            shooter = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
            motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight,
                    intake, transfer, shooter};
            MotorSafety.stopAndBrake(motors);
            wedge = hardwareMap.get(Servo.class, Constants.WEDGE_SERVO_NAME);
            shotStopper = hardwareMap.get(Servo.class, "shotStopper");
            for (DcMotorEx motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            frontLeft.setDirection(drive.frontLeftDirection.get());
            frontRight.setDirection(drive.frontRightDirection.get());
            backLeft.setDirection(drive.backLeftDirection.get());
            backRight.setDirection(drive.backRightDirection.get());
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);
            shooter.setDirection(ShooterFlywheel.SHOOTER.reversed
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            telemetry = RobotTelemetry.forOpMode(telemetry);
            telemetry.addLine("After START, open Panels > Configurables > HardwareTuning.");
            telemetry.addLine("Motor power: -1 to 1. Servo position: 0 to 1.");
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
            applyPower(transfer, transferPower);
            applyPower(shooter, shooterPower);
            applyPosition(wedge, wedgePosition);
            applyPosition(shotStopper, shotStopperPosition);
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void stop() {
        zeroPowerSettings();
        MotorSafety.stopAndBrake(motors);
    }

    private static void zeroPowerSettings() {
        frontLeftPower = frontRightPower = backLeftPower = backRightPower = 0;
        intakePower = transferPower = shooterPower = 0;
    }

    private static void applyPower(DcMotorEx motor, double power) {
        motor.setPower(Math.abs(power) <= 1 ? power : 0);
    }

    private static void applyPosition(Servo servo, double position) {
        if (position >= 0 && position <= 1) servo.setPosition(position);
    }

    private void shutdown(RuntimeException failure) {
        try { stop(); }
        catch (RuntimeException stopFailure) { if (failure != stopFailure) failure.addSuppressed(stopFailure); }
        throw failure;
    }
}
