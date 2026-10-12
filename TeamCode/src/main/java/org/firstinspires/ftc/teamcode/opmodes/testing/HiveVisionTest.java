package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HiveBallTracker;
import org.firstinspires.ftc.teamcode.subsystems.HiveBallTracker.Detection;
import org.firstinspires.ftc.teamcode.subsystems.HiveBallTracker.Filter;
import org.firstinspires.ftc.teamcode.subsystems.HiveVisionConfig;

/** Camera-only bench test. Acquires no motors, servos, drivetrain, or odometry hardware. */
@TeleOp(name = "Hive Vision Test", group = "Diagnostics")
public class HiveVisionTest extends OpMode {
    private Limelight3A camera;
    private HiveBallTracker tracker;
    private int pipeline;
    private boolean stopped = true;

    @Override public void init() {
        stopped = false;
        try {
            telemetry = withDashboard(telemetry);
            telemetry.setMsTransmissionInterval(100);
            pipeline = HiveVisionConfig.pipelineIndex;
            tracker = new HiveBallTracker(pipeline);
            if (HiveVisionConfig.pollRateHz < 1 || HiveVisionConfig.pollRateHz > 250)
                throw new IllegalArgumentException("Hive Vision pollRateHz must be 1..250");
            camera = hardwareMap.get(Limelight3A.class, HiveVisionConfig.hardwareName);
            camera.setPollRateHz(HiveVisionConfig.pollRateHz);
            if (!camera.pipelineSwitch(pipeline))
                throw new IllegalStateException("Could not request Hive Vision detector pipeline " + pipeline);
            camera.start();
            gamepad1.resetEdgeDetection();
        } catch (RuntimeException failure) {
            cleanupAfterFailure(failure);
            throw failure;
        }
    }

    protected Telemetry withDashboard(Telemetry driverTelemetry) {
        return new MultipleTelemetry(driverTelemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override public void init_loop() { sampleAndDisplay(); }
    @Override public void loop() { sampleAndDisplay(); }

    private void sampleAndDisplay() {
        if (stopped) return;
        try {
            if (gamepad1.dpad_up) tracker.setFilter(Filter.RED);
            else if (gamepad1.dpad_down) tracker.setFilter(Filter.BLUE);
            else if (gamepad1.dpad_left) tracker.setFilter(Filter.YELLOW);
            else if (gamepad1.dpad_right) tracker.setFilter(Filter.ALL);
            if (gamepad1.xWasPressed()) tracker.reset();
            LLResult frame = camera.getLatestResult();
            tracker.update(frame);
            telemetry.addLine("Stationary camera test. Dashboard: HiveVisionConfig; STOP/re-INIT applies settings.");
            telemetry.addLine("D-pad: up red, down blue, left yellow, right all. X: reacquire on next frame.");
            telemetry.addData("Vision status", tracker.getStatus());
            telemetry.addData("Pipeline requested / received", pipeline + " / " + (frame == null ? "none" : frame.getPipelineIndex()));
            telemetry.addData("Pipeline type", frame == null ? "none" : frame.getPipelineType());
            telemetry.addData("Filter", tracker.getFilter());
            telemetry.addData("Exposure age ms", tracker.getFrameAgeMs());
            telemetry.addData("Distinct frames", tracker.getDistinctFrames());
            telemetry.addData("Confirmation", tracker.getConfirmationStreak() + " / " + tracker.getRequiredFrames());
            int accepted = 0, index = 0;
            for (Detection detection : tracker.getDetections()) {
                if (detection.rejection.isEmpty()) accepted++;
                if (index < 6) telemetry.addData("Detection " + index,
                        "%s id=%d rawConf=%.3f conf=%.3f tx/ty=%.1f/%.1f area=%.2f %s",
                        detection.className, detection.classId, detection.rawConfidence, detection.confidence,
                        detection.txDegrees, detection.tyDegrees, detection.area,
                        detection.rejection.isEmpty() ? "eligible" : detection.rejection);
                index++;
            }
            telemetry.addData("Eligible / rejected detections", accepted + " / " + (index - accepted));
            if (index > 6) telemetry.addLine("First 6 detections shown; all detections participate in selection.");
            Detection target = tracker.getTarget();
            telemetry.addData("Selected target", target == null ? "none" : target.className);
            if (target != null) {
                telemetry.addData("Target optical tx / ty deg", "%.2f / %.2f", target.txDegrees, target.tyDegrees);
                telemetry.addData("Target confidence", target.confidence);
                if (target.position != null) {
                    telemetry.addData("Ball robot forward / left in", "%.2f / %.2f",
                            target.position.forwardInches, target.position.leftInches);
                    telemetry.addData("Ball camera ground range in", "%.2f", target.position.cameraRangeInches);
                } else if (tracker.hasGeometry()) telemetry.addLine("Selected ray is outside the configured floor working area.");
            }
            if (!tracker.hasGeometry()) telemetry.addLine("Distance unavailable: measure camera/ball geometry before enabling geometryVerified.");
            telemetry.update();
        } catch (RuntimeException failure) {
            cleanupAfterFailure(failure);
            throw failure;
        }
    }

    private void cleanupAfterFailure(RuntimeException failure) {
        try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        if (tracker != null) tracker.stop();
        if (camera != null) camera.stop();
    }
}
