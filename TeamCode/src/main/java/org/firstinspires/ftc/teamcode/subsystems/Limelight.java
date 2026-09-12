package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.VisionFrame;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.util.function.LongSupplier;
import static com.pedropathing.ivy.commands.Commands.instant;

/** Nonblocking camera polling and calibrated, operator-requested stationary pose correction. */
@Config
public class Limelight {
    /** Read at INIT. Configure this AprilTag pipeline and camera extrinsics in the camera UI. */
    public static int pipelineIndex = 0;
    public static double maxStalenessMs = 100;
    /** Enable only after checking the tag map, camera extrinsics and field frame on hardware. */
    public static boolean enableRelocalization = false;
    public static double frameOriginXInches = 0;
    public static double frameOriginYInches = 0;
    public static double frameRotationDegrees = 0;
    public static double maxCorrectionInches = 12;
    public static double maxStationarySpeed = 1;
    public static double maxStationaryTurnDegreesPerSecond = 10;
    public static double maxRobotHeightMeters = 0.5;
    public static double maxRobotTiltDegrees = 15;

    private final Limelight3A camera;
    private final Drivetrain drivetrain;
    private final Telemetry telemetry;
    private final OrientationPublisher orientationPublisher;
    private final VisionResultGate results;
    private final int requestedPipeline;
    private final LongSupplier clock;
    private boolean stationarySampleSeen;
    private long stationarySinceNanos;
    private long lastStationarySampleNanos;
    private String lastCorrection = "Disabled until field/camera calibration is verified";

    public Limelight(HardwareMap hardwareMap, Drivetrain drivetrain, Telemetry telemetry) {
        this(hardwareMap.get(Limelight3A.class, "limelight"), drivetrain, telemetry, pipelineIndex);
    }

    private Limelight(Limelight3A camera, Drivetrain drivetrain, Telemetry telemetry, int pipeline) {
        this(camera, drivetrain, telemetry, pipeline,
                new OrientationPublisher(() -> camera.pipelineSwitch(pipeline), camera::updateRobotOrientation),
                System::nanoTime);
    }

    Limelight(Limelight3A camera, Drivetrain drivetrain, Telemetry telemetry, int pipeline,
              OrientationPublisher publisher, LongSupplier clock) {
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;
        this.clock = clock;
        requestedPipeline = pipeline;
        orientationPublisher = publisher;
        results = new VisionResultGate(pipeline, clock);
        camera.setPollRateHz(100);
        camera.start();
    }

    public LLResult getFreshResult() {
        return orientationPublisher.isReady() ? results.getFresh(maxStalenessMs) : null;
    }

    public String getLastCorrection() { return lastCorrection; }

    /** Takes drive ownership and, after validation, corrects stationary translation only. */
    public Command relocalize() {
        if (!enableRelocalization) return instant(() -> lastCorrection = "Relocalization disabled");
        return instant(this::correctTranslation).requiring(drivetrain);
    }

    private void correctTranslation() {
        if (!enableRelocalization) { lastCorrection = "Relocalization disabled"; return; }
        if (!drivetrain.isLocalizationReady()) {
            stationarySampleSeen = false;
            lastCorrection = "Pinpoint is not ready";
            return;
        }
        LLResult result = getFreshResult();
        if (result == null) { lastCorrection = "No fresh result from requested pipeline"; return; }
        if (!orientationPublisher.hasRecentHeading(maxStalenessMs)) { lastCorrection = "Heading feed is stale"; return; }
        if (result.getBotposeTagCount() < 1) { lastCorrection = "No tags used in pose"; return; }
        if (!validLimits()) {
            stationarySampleSeen = false;
            lastCorrection = "Invalid correction limits";
            return;
        }

        Pose current = drivetrain.getPose();
        Pose velocity = drivetrain.getVelocity();
        if (!finite(current) || !finite(velocity)) {
            stationarySampleSeen = false;
            lastCorrection = "Invalid odometry sample";
            return;
        }
        if (Math.hypot(velocity.getX(), velocity.getY()) > maxStationarySpeed
                || Math.abs(Math.toDegrees(velocity.getHeading())) > maxStationaryTurnDegreesPerSecond) {
            stationarySampleSeen = false;
            lastCorrection = "Keep the robot stationary for correction";
            return;
        }
        if (!stationaryForFrameDelay()) {
            lastCorrection = "Keep stationary for the full camera delay before correction";
            return;
        }
        try {
            // The SDK returns a zero Pose3D if botpose_orb is missing. Check raw presence first.
            JSONArray raw = new JSONObject(result.toString()).optJSONArray("botpose_orb");
            if (raw == null || raw.length() < 6) { lastCorrection = "No MegaTag2 pose payload"; return; }
            double[] pose = new double[6];
            for (int i = 0; i < pose.length; i++) {
                pose[i] = raw.getDouble(i);
                if (!Double.isFinite(pose[i])) { lastCorrection = "Nonfinite camera pose"; return; }
            }
            if (Math.abs(pose[2]) > maxRobotHeightMeters || Math.abs(pose[3]) > maxRobotTiltDegrees
                    || Math.abs(pose[4]) > maxRobotTiltDegrees) {
                lastCorrection = "Camera pose height/tilt rejected";
                return;
            }
            Pose candidate = frame().toPedro(pose[0], pose[1], pose[5]);
            if (Math.hypot(candidate.getX() - current.getX(), candidate.getY() - current.getY()) > maxCorrectionInches) {
                lastCorrection = "Correction exceeds allowed distance";
                return;
            }
            // MegaTag2 yaw depends on our supplied heading; keep the independent odometry state.
            drivetrain.applyVisionTranslation(new Pose(candidate.getX(), candidate.getY(), current.getHeading()));
            lastCorrection = "Translation corrected; odometry heading retained";
        } catch (JSONException | IllegalArgumentException invalidPose) {
            lastCorrection = "Invalid camera payload or frame configuration";
        }
    }

    private static VisionFrame frame() {
        return new VisionFrame(frameOriginXInches, frameOriginYInches, frameRotationDegrees);
    }

    private static boolean finite(Pose pose) {
        return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getHeading());
    }

    private static boolean validLimits() {
        for (double value : new double[]{maxCorrectionInches, maxStationarySpeed,
                maxStationaryTurnDegreesPerSecond, maxRobotHeightMeters, maxRobotTiltDegrees}) {
            if (!Double.isFinite(value) || value < 0) return false;
        }
        return true;
    }

    private void observeStationarity() {
        Pose pose = drivetrain.getPose();
        Pose velocity = drivetrain.getVelocity();
        if (!drivetrain.isLocalizationReady() || !finite(pose) || !finite(velocity)
                || !validLimits() || !Double.isFinite(maxStalenessMs) || maxStalenessMs <= 0
                || Math.hypot(velocity.getX(), velocity.getY()) > maxStationarySpeed
                || Math.abs(Math.toDegrees(velocity.getHeading())) > maxStationaryTurnDegreesPerSecond) {
            stationarySampleSeen = false;
            return;
        }
        long now = clock.getAsLong();
        double gapMs = (now - lastStationarySampleNanos) / 1e6;
        if (!stationarySampleSeen || gapMs < 0 || gapMs > maxStalenessMs) {
            stationarySinceNanos = now;
            stationarySampleSeen = true;
        }
        lastStationarySampleNanos = now;
    }

    private boolean stationaryForFrameDelay() {
        if (!stationarySampleSeen) return false;
        double gapMs = (clock.getAsLong() - lastStationarySampleNanos) / 1e6;
        if (!Double.isFinite(maxStalenessMs) || maxStalenessMs <= 0 || gapMs < 0 || gapMs > maxStalenessMs) {
            stationarySampleSeen = false;
            return false;
        }
        // Count only the interval covered by observed stationary samples, not time since one old read.
        return (lastStationarySampleNanos - stationarySinceNanos) / 1e6 >= maxStalenessMs;
    }

    public void periodic() {
        observeStationarity();
        if (drivetrain.isLocalizationReady()) {
            try {
                orientationPublisher.publish(frame().toCameraHeading(drivetrain.getHeading()));
            } catch (IllegalArgumentException badFrame) {
                lastCorrection = "Invalid vision frame; heading feed paused";
            }
        }
        LLResult received = camera.getLatestResult();
        results.observe(received);
        LLResult fresh = getFreshResult();
        telemetry.addData("Limelight Expected Pipeline", requestedPipeline);
        telemetry.addData("Limelight Pipeline Ready", orientationPublisher.isReady());
        telemetry.addData("Limelight Heading Fresh", orientationPublisher.hasRecentHeading(maxStalenessMs));
        telemetry.addData("Limelight Fresh", fresh != null);
        telemetry.addData("Limelight Correction Enabled", enableRelocalization);
        telemetry.addData("Limelight Last Correction", lastCorrection);
        if (received != null) telemetry.addData("Limelight Actual Pipeline", received.getPipelineIndex());
        if (fresh != null) {
            telemetry.addData("Limelight Tags Used", fresh.getBotposeTagCount());
            telemetry.addData("Limelight Tx/Ty", "%.1f / %.1f", fresh.getTx(), fresh.getTy());
            telemetry.addData("Limelight Raw MT2", fresh.getBotpose_MT2());
        }
    }

    public void stop() {
        stationarySampleSeen = false;
        orientationPublisher.close();
        results.clear();
        camera.stop();
    }
}
