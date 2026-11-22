package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

import java.util.Arrays;
import java.util.HashSet;

/** Tune two flywheel motors independently using the encoder on each motor. */
@Configurable
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuning extends OpMode {
    public enum ControlMode { POWER, RPM }

    public static ShooterFlywheel.SpeedTuning speed = ShooterFlywheel.SHOOTER;
    public static ShooterFlywheel.SpeedTuning speed2 = new ShooterFlywheel.SpeedTuning(0.04, 0.001, false);
    public static volatile ControlMode controlMode = ControlMode.RPM;
    public static volatile double power = 0.25, power2;
    public static volatile double intakePower;
    public static volatile int shotStopperBlocked = 1;

    private TuningMotor first, second;
    private DcMotorEx intake;
    private Servo shotStopper;
    private boolean ready;

    @Override public void init() {
        if (ready) stop();
        first = second = null;
        intake = null;
        shotStopper = null;
        resetAuxiliaryControls();

        try {
            DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, Constants.SHOOTER_MOTOR_NAME);
            DcMotorEx motor2 = hardwareMap.tryGet(DcMotorEx.class, Constants.SECOND_SHOOTER_MOTOR_NAME);
            DcMotorEx mappedIntake = hardwareMap.get(DcMotorEx.class, Constants.INTAKE_MOTOR_NAME);
            if (new HashSet<>(Arrays.asList(motor1, motor2, mappedIntake)).size() != 3) {
                throw new IllegalArgumentException("Configure separate ports for each flywheel and the intake/transfer motor.");
            }
            shotStopper = hardwareMap.tryGet(Servo.class, "shotStopper");
            if (shotStopper != null) shotStopper.setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
            intake = mappedIntake;
            MotorSafety.stopAndBrake(intake);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setDirection(Constants.INTAKE_MOTOR_DIRECTION);

            first = new TuningMotor(motor1, ShooterFlywheel.forTuning(
                    motor1, hardwareMap, Constants.SHOOTER_MOTOR_NAME, speed), "shooter.", "");
            if (motor2 != null) {
                second = new TuningMotor(motor2, ShooterFlywheel.forTuning(
                        motor2, hardwareMap, Constants.SECOND_SHOOTER_MOTOR_NAME, speed2), "shooter2.", "Motor 2 ");
            }

            telemetry = RobotTelemetry.forOpMode(telemetry);
            telemetry.addLine("Panels > Configurables > FlywheelTuning");
            telemetry.addLine("Motor 1 uses speed.setpointRpm. Motor 2 uses speed2.setpointRpm.");
            if (shotStopper != null) telemetry.addLine("Set shotStopperBlocked to 1 to block or 0 to unblock.");
            reportAvailability();
            telemetry.update();
            ready = true;
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void init_loop() { loop(); }

    @Override public void loop() {
        if (!ready) return;
        try {
            ControlMode mode = controlMode;
            applyTuning(first, mode, speed, power);
            if (second != null) applyTuning(second, mode, speed2, power2);
            if (shotStopper != null) {
                applyPower(intake, intakePower);
                boolean blocked = shotStopperBlocked != 0;
                shotStopper.setPosition(blocked
                        ? Constants.SHOT_STOPPER_BLOCKED_POSITION : Constants.SHOT_STOPPER_UNBLOCKED_POSITION);
                telemetry.addData("shotStopper.blocked", blocked);
            }
            reportAvailability();
            telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void applyTuning(TuningMotor channel, ControlMode mode, ShooterFlywheel.SpeedTuning tuning, double power) {
        ShooterFlywheel flywheel = channel.flywheel;
        DcMotorEx motor = channel.motor;
        boolean directionReady = flywheel.prepareTuningDirection(tuning.reversed);
        double targetRpm = directionReady && mode == ControlMode.RPM ? tuning.setpointRpm : 0;
        double measuredRpm;
        if (targetRpm != 0) {
            flywheel.updateTuningRpm(targetRpm, tuning);
            measuredRpm = flywheel.getMode() == ShooterFlywheel.Mode.OFF
                    ? flywheel.ticksToRpm(motor.getVelocity()) : flywheel.getMeasuredRpm();
        } else {
            if (flywheel.getMode() != ShooterFlywheel.Mode.OFF) flywheel.idle();
            double requestedPower = directionReady && mode == ControlMode.POWER ? power : 0;
            applyPower(motor, requestedPower);
            measuredRpm = flywheel.ticksToRpm(motor.getVelocity());
        }
        String status = mode == ControlMode.POWER ? "Power tuning" : flywheel.getStatus();
        if (!directionReady) status = "Braking for direction change";
        else if (mode == ControlMode.RPM && targetRpm == 0) status = "RPM target is zero";
        telemetry.addData(channel.telemetryPrefix + "controlMode", mode);
        telemetry.addData(channel.telemetryPrefix + "direction", motor.getDirection());
        // Panels Graph reads numeric series from the same packets as the Telemetry widget.
        telemetry.addData(channel.rpmLabelPrefix + "Target RPM", targetRpm);
        telemetry.addData(channel.rpmLabelPrefix + "Current RPM", measuredRpm);
        telemetry.addData(channel.telemetryPrefix + "power", motor.getPower());
        telemetry.addData(channel.telemetryPrefix + "atSpeed", flywheel.isAtSpeed());
        telemetry.addData(channel.telemetryPrefix + "status", status);
        if (mode == ControlMode.RPM) {
            telemetry.addData(channel.telemetryPrefix + "requestedVolts", flywheel.getRequestedVolts());
            telemetry.addData(channel.telemetryPrefix + "appliedVolts", flywheel.getAppliedVolts());
            telemetry.addData(channel.telemetryPrefix + "outputLimited", flywheel.isOutputLimited());
        }
    }

    @Override public void stop() {
        ready = false;
        resetAuxiliaryControls();
        try {
            try { if (first != null) first.flywheel.stop(); }
            finally { if (second != null) second.flywheel.stop(); }
        } finally {
            try { MotorSafety.stopAndBrake(intake); }
            finally {
                if (shotStopper != null) shotStopper.setPosition(Constants.SHOT_STOPPER_BLOCKED_POSITION);
            }
        }
    }

    private static void resetAuxiliaryControls() {
        intakePower = 0;
        shotStopperBlocked = 1;
    }

    private void reportAvailability() {
        if (second == null) telemetry.addLine("Motor 2 unavailable. Configure shooter2 on Control Hub port 2.");
        if (shotStopper == null) telemetry.addLine("Shot stopper unavailable. Flywheels only. Remove balls before INIT.");
    }

    private static void applyPower(DcMotorEx motor, double power) {
        motor.setPower(Math.abs(power) <= 1 ? power : 0);
    }

    private void shutdown(RuntimeException failure) {
        try { stop(); }
        catch (RuntimeException stopFailure) { if (failure != stopFailure) failure.addSuppressed(stopFailure); }
        throw failure;
    }

    private static final class TuningMotor {
        final DcMotorEx motor;
        final ShooterFlywheel flywheel;
        final String telemetryPrefix;
        final String rpmLabelPrefix;

        TuningMotor(DcMotorEx motor, ShooterFlywheel flywheel, String telemetryPrefix, String rpmLabelPrefix) {
            this.motor = motor;
            this.flywheel = flywheel;
            this.telemetryPrefix = telemetryPrefix;
            this.rpmLabelPrefix = rpmLabelPrefix;
        }
    }
}
