package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

/** Shared power and RPM tuning for one shooter with manual ball feed controls. */
abstract class ShooterTuningOpMode extends OpMode {
    public enum ControlMode { POWER, RPM }

    private final String motorName;
    private final String telemetryPrefix;
    private DcMotorEx motor, intake, transfer;
    private ShooterFlywheel flywheel;
    private Servo shotStopper;

    ShooterTuningOpMode(String motorName, String telemetryName) {
        this.motorName = motorName;
        telemetryPrefix = telemetryName + ".";
    }

    abstract ShooterFlywheel.SpeedTuning speedTuning();
    abstract void resetAuxiliaryControls();
    abstract void updateTuning();

    @Override public void init() {
        resetAuxiliaryControls();
        try {
            shotStopper = hardwareMap.get(Servo.class, "shotStopper");
            shotStopper.setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
            motor = hardwareMap.get(DcMotorEx.class, motorName);
            flywheel = ShooterFlywheel.forTuning(motor, hardwareMap, motorName, speedTuning());
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            transfer = hardwareMap.get(DcMotorEx.class, Constants.TRANSFER_MOTOR_NAME);
            MotorSafety.stopAndBrake(intake, transfer);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry = RobotTelemetry.forOpMode(telemetry);
            telemetry.addLine("Open Panels > Configurables > " + getClass().getSimpleName() + ".");
            telemetry.addLine("POWER uses power. RPM uses speed.setpointRpm. Set kP to 0 for feedforward tuning.");
            telemetry.addLine("Set shotStopperBlocked to 1 to block or 0 to unblock.");
            telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void init_loop() { loop(); }

    @Override public void loop() {
        try { updateTuning(); }
        catch (RuntimeException failure) { shutdown(failure); }
    }

    void applyTuning(ControlMode mode, double power, double intakePower, double transferPower, int shotStopperBlocked) {
        ShooterFlywheel.SpeedTuning speed = speedTuning();
        boolean directionReady = flywheel.prepareTuningDirection(speed.reversed);
        double targetRpm = directionReady && mode == ControlMode.RPM ? speed.setpointRpm : 0;
        double measuredRpm;
        if (targetRpm != 0) {
            flywheel.updateTuningRpm(targetRpm, speed);
            measuredRpm = flywheel.getMode() == ShooterFlywheel.Mode.OFF
                    ? flywheel.ticksToRpm(motor.getVelocity()) : flywheel.getMeasuredRpm();
        } else {
            if (flywheel.getMode() != ShooterFlywheel.Mode.OFF) flywheel.idle();
            double requestedPower = directionReady && mode == ControlMode.POWER ? power : 0;
            applyPower(motor, requestedPower);
            measuredRpm = flywheel.ticksToRpm(motor.getVelocity());
        }
        applyPower(intake, intakePower);
        applyPower(transfer, transferPower);
        boolean blocked = shotStopperBlocked != 0;
        shotStopper.setPosition(blocked
                ? Constants.SHOT_STOPPER_BLOCKED_POSITION : Constants.SHOT_STOPPER_UNBLOCKED_POSITION);
        telemetry.addData("shotStopper.blocked", blocked);
        String status = mode == ControlMode.POWER ? "Power tuning" : flywheel.getStatus();
        if (!directionReady) status = "Braking for direction change";
        else if (mode == ControlMode.RPM && targetRpm == 0) status = "RPM target is zero";
        telemetry.addData(telemetryPrefix + "controlMode", mode);
        telemetry.addData(telemetryPrefix + "direction", motor.getDirection());
        telemetry.addData(telemetryPrefix + "setpointRpm", targetRpm);
        telemetry.addData(telemetryPrefix + "measuredRpm", measuredRpm);
        telemetry.addData(telemetryPrefix + "power", motor.getPower());
        telemetry.addData(telemetryPrefix + "atSpeed", flywheel.isAtSpeed());
        telemetry.addData(telemetryPrefix + "status", status);
        if (mode == ControlMode.RPM) {
            telemetry.addData(telemetryPrefix + "requestedVolts", flywheel.getRequestedVolts());
            telemetry.addData(telemetryPrefix + "appliedVolts", flywheel.getAppliedVolts());
            telemetry.addData(telemetryPrefix + "outputLimited", flywheel.isOutputLimited());
        }
        telemetry.addLine("Set power, the RPM target, or both gains to 0 to stop the active control.");
        telemetry.update();
    }

    @Override public void stop() {
        resetAuxiliaryControls();
        try { if (flywheel != null) flywheel.stop(); }
        finally {
            try { MotorSafety.stopAndBrake(intake, transfer); }
            finally {
                if (shotStopper != null) shotStopper.setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
            }
        }
    }

    private static void applyPower(DcMotorEx motor, double power) {
        motor.setPower(Math.abs(power) <= 1 ? power : 0);
    }

    private void shutdown(RuntimeException failure) {
        try { stop(); }
        catch (RuntimeException stopFailure) { if (failure != stopFailure) failure.addSuppressed(stopFailure); }
        throw failure;
    }
}
