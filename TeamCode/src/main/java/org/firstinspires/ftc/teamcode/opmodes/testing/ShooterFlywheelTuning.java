package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;

import java.util.Collections;
import java.util.List;

/** Dashboard tuner for two motors on ONE linked flywheel. No hood, drive or camera needed. */
@Config
@TeleOp(name = "Shooter Flywheel Tuning", group = "Prototyping")
public class ShooterFlywheelTuning extends OpMode {
    public static volatile double targetRpm = 1000, characterizationVoltage = 1;
    public static volatile double rpmStep = 100, voltageStep = 0.1;
    private ShooterFlywheel flywheel;
    private List<LynxModule> hubs = Collections.emptyList();
    private boolean active, stopped, releaseRequired;

    public ShooterFlywheelTuning() { }

    ShooterFlywheelTuning(ShooterFlywheel flywheel, Telemetry telemetry) {
        this.flywheel = flywheel; this.telemetry = telemetry;
    }

    @Override public void init() {
        active = stopped = false; releaseRequired = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        try {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            flywheel = new ShooterFlywheel(hardwareMap);
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    @Override public void init_loop() {
        if (stopped) return;
        tick();
        telemetry.addLine("INIT: encoder / battery readout only; verify wiring before START.");
        telemetry.update();
    }

    @Override public void start() {
        if (stopped) return;
        active = true; releaseRequired = true;
        gamepad1.resetEdgeDetection();
        flywheel.enable();
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            // Consume SDK latched edges even while canceled, avoiding delayed setpoint edits.
            boolean up = gamepad1.dpadUpWasPressed(), down = gamepad1.dpadDownWasPressed();
            boolean left = gamepad1.dpadLeftWasPressed(), right = gamepad1.dpadRightWasPressed();
            if (gamepad1.x && !gamepad1.left_bumper) releaseRequired = true;
            if (releaseRequired) {
                flywheel.idle();
                if (controlsReleased()) releaseRequired = false;
            } else {
                if (Double.isFinite(rpmStep) && rpmStep > 0 && rpmStep <= 500) {
                    if (up) targetRpm = Math.min(ShooterFlywheel.MAX_MOTOR_RPM, targetRpm + rpmStep);
                    if (down) targetRpm = Math.max(0, targetRpm - rpmStep);
                }
                if (Double.isFinite(voltageStep) && voltageStep > 0 && voltageStep <= 0.5) {
                    if (left) characterizationVoltage = Math.max(0, characterizationVoltage - voltageStep);
                    if (right) characterizationVoltage = Math.min(12, characterizationVoltage + voltageStep);
                }
                if (gamepad1.left_bumper) {
                    if (gamepad1.x && !gamepad1.y && !gamepad1.a) flywheel.requestMotorTest(true);
                    else if (gamepad1.y && !gamepad1.x && !gamepad1.a) flywheel.requestMotorTest(false);
                    else if (gamepad1.a && !gamepad1.x && !gamepad1.y) flywheel.requestVoltage(characterizationVoltage);
                    else flywheel.idle();
                } else if (gamepad1.right_trigger > 0.25) flywheel.requestRpm(targetRpm);
                else flywheel.idle();
            }
            tick();
            if (releaseRequired) telemetry.addLine("Release RT, LB and A/B/X/Y to arm controls.");
            telemetry.update();
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    private boolean controlsReleased() {
        return gamepad1.right_trigger <= 0.25 && !gamepad1.left_bumper
                && !gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y;
    }

    private void tick() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        flywheel.periodic();
        telemetry.addData("Mode / status", flywheel.getMode() + " / " + flywheel.getStatus());
        telemetry.addData("flywheel.targetRpm", targetRpm);
        telemetry.addData("flywheel.referenceRpm", flywheel.getReferenceRpm());
        telemetry.addData("flywheel.referenceRpmPerSec", flywheel.getReferenceAccelerationRpmPerSecond());
        telemetry.addData("flywheel.measuredRpm", flywheel.getMeasuredRpm());
        telemetry.addData("Feedback encoder", flywheel.getFeedbackEncoder());
        telemetry.addData("flywheel.feedforwardVolts", flywheel.getFeedforwardVolts());
        telemetry.addData("flywheel.feedbackVolts", flywheel.getFeedbackVolts());
        telemetry.addData("flywheel.requestedVolts", flywheel.getRequestedVolts());
        telemetry.addData("flywheel.appliedVolts", flywheel.getAppliedVolts());
        telemetry.addData("flywheel.motorPower", flywheel.getPower());
        telemetry.addData("flywheel.batteryVolts", flywheel.getBatteryVolts());
        telemetry.addData("Output limited / at speed", flywheel.isOutputLimited() + " / " + flywheel.isAtSpeed());
        telemetry.addData("Characterization voltage request", characterizationVoltage);
        telemetry.addLine("Dashboard: ShooterFlywheel = gains/limits; ShooterFlywheelTuning = RPM/voltage targets.");
        telemetry.addLine("RT: hold RPM | LB+A: shared voltage | LB+X/Y: left/right direction test | X: cancel");
        telemetry.addLine("D-pad up/down: RPM | left/right: characterization volts | release: coast + clear fault");
        telemetry.addLine("Practice only. Both motors drive ONE flywheel. At speed is not automatic feed permission.");
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true; active = false;
        if (flywheel != null) flywheel.stop();
    }
}
