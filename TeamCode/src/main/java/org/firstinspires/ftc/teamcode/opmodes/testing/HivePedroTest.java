package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.HiveBallTracker;
import org.firstinspires.ftc.teamcode.subsystems.HivePedroController;
import org.firstinspires.ftc.teamcode.subsystems.HiveVisionConfig;

/** Standalone drive practice; one Limelight owner, no launcher/intake/turret hardware. */
@NextTeleop(name = "Hive Pedro Test", group = "Diagnostics")
public class HivePedroTest extends RobotOpMode {
    private Limelight3A camera;
    private HiveBallTracker tracker;
    private HivePedroController approach;
    private int pipeline;

    public HivePedroTest(Robot robot) { super(robot); }

    @Override protected void onInit() {
        // DRIVE_ONLY creates the chassis; this test alone owns the detector camera/pipeline.
        robot.drivetrain.setStartingPose(Pose.zero());
        pipeline = HiveVisionConfig.pipelineIndex;
        tracker = new HiveBallTracker(pipeline);
        approach = new HivePedroController(robot.drivetrain, tracker);
        if (HiveVisionConfig.pollRateHz < 1 || HiveVisionConfig.pollRateHz > 250)
            throw new IllegalArgumentException("Hive Vision pollRateHz must be 1..250");
        camera = robot.hardwareMap.get(Limelight3A.class, HiveVisionConfig.hardwareName);
        camera.setPollRateHz(HiveVisionConfig.pollRateHz);
        if (!camera.pipelineSwitch(pipeline)) throw new IllegalStateException("Could not select Hive Vision pipeline " + pipeline);
        camera.start();
    }

    @Override protected void onInitLoop() {
        sampleCamera();
        if (gamepad1.xWasPressed()) robot.drivetrain.recalibrateLocalization();
        robot.telemetry.addLine("INIT: keep still. X calibrates Pinpoint. Verify fixed camera geometry and Pedro tuning.");
        showTelemetry();
    }

    @Override protected void onStart() { approach.start(); }

    @Override protected void onLoop() {
        sampleCamera();
        boolean stickMoved = Math.abs(gamepad1.left_stick_x) > 0.15 || Math.abs(gamepad1.left_stick_y) > 0.15
                || Math.abs(gamepad1.right_stick_x) > 0.15 || Math.abs(gamepad1.right_stick_y) > 0.15;
        approach.beforeDrive(gamepad1.right_trigger > 0.5, gamepad1.x || stickMoved);
    }

    @Override protected void onAfterMechanisms() { approach.afterSensors(); showTelemetry(); }

    private void sampleCamera() {
        HiveBallTracker.Filter old = tracker.getFilter();
        if (gamepad1.dpad_up) tracker.setFilter(HiveBallTracker.Filter.RED);
        else if (gamepad1.dpad_down) tracker.setFilter(HiveBallTracker.Filter.BLUE);
        else if (gamepad1.dpad_left) tracker.setFilter(HiveBallTracker.Filter.YELLOW);
        // One measured ball height per run; this moving test deliberately has no mixed-class mode.
        if (old != tracker.getFilter()) approach.abort("Class changed; release RT to reacquire");
        LLResult frame = camera.getLatestResult();
        tracker.update(frame);
        robot.telemetry.addData("Detector pipeline requested / received", pipeline + " / " + (frame == null ? "none" : frame.getPipelineIndex()));
    }

    private void showTelemetry() {
        robot.telemetry.addLine("Release then hold RT: one approach. Release RT / X / move a stick: stop.");
        robot.telemetry.addLine("D-pad left/up/down: yellow/red/blue. Settings apply on re-INIT. No intake/pickup action.");
        robot.telemetry.addData("Approach phase", approach.getPhase());
        robot.telemetry.addData("Approach status", approach.getStatus());
        robot.telemetry.addData("Pedro configured", robot.drivetrain.isPathFollowingConfigured());
        robot.telemetry.addData("Camera geometry verified", tracker.hasGeometry());
        robot.telemetry.addData("Ball filter / tracker", tracker.getFilter() + " / " + tracker.getStatus());
        robot.telemetry.addData("Ball frame age ms", tracker.getFrameAgeMs());
        robot.telemetry.addData("Ball confirmation", tracker.getConfirmationStreak() + " / " + tracker.getRequiredFrames());
        robot.telemetry.addData("Travel used in", approach.getTraveledInches());
        robot.telemetry.addData("Planned Pedro endpoint", approach.getPlannedEnd());
        HiveBallTracker.Detection target = tracker.getTarget();
        if (target != null && target.position != null) robot.telemetry.addData("Ball forward / left in",
                "%.1f / %.1f", target.position.forwardInches, target.position.leftInches);
    }

    @Override protected void onStop() {
        try { if (approach != null) approach.stop(); }
        finally {
            if (tracker != null) tracker.stop();
            if (camera != null) camera.stop();
        }
    }
}
