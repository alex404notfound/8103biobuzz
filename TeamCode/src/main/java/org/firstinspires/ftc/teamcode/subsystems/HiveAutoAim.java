package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.math.AimPoseHistory;
import org.firstinspires.ftc.teamcode.math.HiveAimGeometry;
import org.firstinspires.ftc.teamcode.math.HiveAimGeometry.Vec;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.LongSupplier;

/** Turret-mounted camera + short-term Pinpoint motion compensation. Azimuth only, never a fire permit. */
@Config
public class HiveAutoAim {
    public static volatile boolean cameraPoseVerified = false, openingGeometryVerified = false;
    public static volatile boolean cellOrientationVerified = false;
    // Inches; turret frame is forward, left, up. Camera optical axes are right, down, forward.
    public static volatile double cameraForward = 0, cameraLeft = 0, cameraUp = 0;
    public static volatile double cameraRollDegrees = 0, cameraPitchDegrees = 0, cameraYawDegrees = 0;
    public static volatile double pivotForward = 0, pivotLeft = 0, turretZeroYawDegrees = 0;
    public static volatile double shooterForward = 0, shooterLeft = 0, shooterYawDegrees = 0;
    // From the center of the sticker row, in TAG axes: printed right, printed down, tag normal.
    // Measure against the real field. The sticker center is on the bottom, not in the opening.
    public static volatile double openingTagX = 0, openingTagY = 0, openingTagZ = 0;
    // Zero normal deliberately prevents aiming, and finite placeholders remain editable in Dashboard.
    public static volatile double openingNormalTagX = 0, openingNormalTagY = 0, openingNormalTagZ = 0;
    // Manual mechanical tilt: verify the transformed printed-up direction on both real states.
    public static volatile double upPitchDegrees = 30, downPitchDegrees = -30, pitchToleranceDegrees = 8;
    public static volatile double pitchAgreementDegrees = 4, centerAgreementInches = 2;
    public static volatile double normalAgreementDegrees = 8;
    public static volatile double minimumTagArea = 0.02, maxFrameAgeMs = 150, extraLatencyMs = 0;
    public static volatile double stableDwellMs = 200, stablePitchDegrees = 3, stablePositionInches = 2;
    // Zero disables blind prediction. If enabled, this is a total age since exposure, NOT time since loss.
    public static volatile double maximumPredictionAgeMs = 0;
    public static volatile double minimumRangeInches = 12, maximumRangeInches = 180;
    // Heuristic approach filter only. It does not model gravity, the rim, ball diameter or shot spread.
    public static volatile double minimumApproachDegrees = 30;

    public enum CellState { UNKNOWN, UP, DOWN, TRANSITION }
    private final LongSupplier clock;
    private final AimPoseHistory history = new AimPoseHistory();
    private final VisionResultGate frames;
    private HiveCell selectedCell;
    private Alliance selectedAlliance;
    private double[] configuration;
    private double lastFrame = Double.NaN, pitch = Double.NaN, ageMs = Double.NaN;
    private double goal = Double.NaN, range = Double.NaN, approach = Double.NaN;
    private long captureTime, stableSince, lastNewFrame;
    private int stableFrames, tagCount;
    private Vec targetField, normalField, stabilityAnchor;
    private double stablePitch;
    private boolean usable, predicted;
    private CellState state = CellState.UNKNOWN;
    private String status = "Waiting for a referenced turret and healthy Pinpoint";

    public HiveAutoAim(int pipeline) { this(pipeline, System::nanoTime); }
    HiveAutoAim(int pipeline, LongSupplier clock) {
        this.clock = clock;
        frames = new VisionResultGate(pipeline, clock);
    }

    /** All angles except headingRadians use degrees. Call once per loop after reading sensors. */
    public void update(LLResult result, double exposureAgeMs, boolean healthy, double x, double y,
                       double headingRadians, boolean turretReferenced, double turretDegrees,
                       Alliance alliance, HiveCell cell) {
        usable = false; predicted = false; goal = range = approach = Double.NaN;
        long now = clock.getAsLong();
        double[] snapshot = configSnapshot();
        if (cell != selectedCell || alliance != selectedAlliance || !Arrays.equals(snapshot, configuration)) {
            invalidate(); lastFrame = Double.NaN;
            selectedCell = cell; selectedAlliance = alliance; configuration = snapshot;
        }
        if (!healthy || !turretReferenced || !finite(x, y, headingRadians, turretDegrees)) {
            clear(); status = "Healthy Pinpoint and a physical turret zero required"; return;
        }
        if (!history.add(now, x, y, headingRadians, turretDegrees)) {
            invalidate(); status = "Pose history interrupted; collecting new samples"; return;
        }
        if (cell == null || alliance == null || cell.alliance != alliance) {
            invalidate(); status = "Select a CELL on your alliance's HIVE"; return;
        }
        if (!validConfig()) { invalidate(); status = "Invalid aim settings"; return; }
        frames.observe(result);
        LLResult fresh = frames.getFresh(maxFrameAgeMs);
        boolean freshObserved = fresh != null && Double.isFinite(exposureAgeMs) && exposureAgeMs >= 0
                && exposureAgeMs + extraLatencyMs <= maxFrameAgeMs;
        ageMs = freshObserved ? exposureAgeMs + extraLatencyMs : Double.NaN;
        if (freshObserved && Double.compare(lastFrame, fresh.getTimestamp()) != 0) {
            if (fresh.getTimestamp() < lastFrame) invalidate();
            lastFrame = fresh.getTimestamp();
            long imageTime = now - (long) ((exposureAgeMs + extraLatencyMs) * 1e6);
            AimPoseHistory.Sample atImage = history.interpolate(imageTime);
            if (atImage == null) { invalidate(); status = "No pose samples bracketing camera exposure"; return; }
            if (!observe(fresh, atImage, now)) return;
            captureTime = imageTime;
        }
        if (targetField == null) return;
        ageMs = (now - captureTime) * 1e-6;
        predicted = !freshObserved;
        double ageLimit = predicted ? maximumPredictionAgeMs : maxFrameAgeMs;
        if (ageMs < 0 || ageMs > ageLimit || (predicted && ageLimit == 0)) {
            invalidate(); status = "Vision expired; turret aim stopped"; return;
        }
        if (stableFrames < 3 || (lastNewFrame - stableSince) * 1e-6 < stableDwellMs) {
            status = "Waiting for consistent UP observations"; return;
        }
        Vec pivot = new Vec(x, y, 0).plus(HiveAimGeometry.rotate(new Vec(pivotForward, pivotLeft, 0),
                0, 0, Math.toDegrees(headingRadians)));
        Vec relative = HiveAimGeometry.rotate(targetField.minus(pivot), 0, 0, -Math.toDegrees(headingRadians));
        double bearing = HiveAimGeometry.turretBearingDegrees(relative, shooterForward, shooterLeft, shooterYawDegrees);
        goal = HiveAimGeometry.nearestTurretDegrees(bearing - turretZeroYawDegrees, turretDegrees,
                AxonTurret.limitTravel, AxonTurret.minDegrees, AxonTurret.maxDegrees);
        range = Math.hypot(relative.x, relative.y);
        if (!finite(bearing, goal)) { status = "No forward muzzle solution inside turret travel"; return; }
        // Horizontal approach to the calibrated OUTWARD mouth normal. A vertical normal is invalid.
        Vec muzzle = pivot.plus(HiveAimGeometry.rotate(new Vec(shooterForward, shooterLeft, 0),
                0, 0, Math.toDegrees(headingRadians) + bearing));
        Vec towardShooter = muzzle.minus(targetField);
        double normalLength = Math.hypot(normalField.x, normalField.y);
        double horizontalRange = Math.hypot(towardShooter.x, towardShooter.y);
        double cosine = (towardShooter.x * normalField.x + towardShooter.y * normalField.y)
                / (horizontalRange * normalLength);
        approach = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, cosine))));
        if (!finite(goal, range, approach) || normalLength < 0.1 || range < minimumRangeInches
                || range > maximumRangeInches || approach < minimumApproachDegrees) {
            status = "Outside travel/range or approach too shallow/behind opening"; return;
        }
        if (!cameraPoseVerified || !cellOrientationVerified || !openingGeometryVerified) {
            status = "Diagnostic geometry only: verify camera, CELL orientation and opening"; return;
        }
        usable = true;
        status = predicted ? "ODOMETRY PRE-AIM ONLY: no current CELL confirmation"
                : "UP cell: azimuth available (trajectory/clearance NOT verified)";
    }

    private boolean observe(LLResult frame, AimPoseHistory.Sample pose, long now) {
        List<Vec> targets = new ArrayList<>(), normals = new ArrayList<>();
        List<Double> pitches = new ArrayList<>();
        Set<Integer> ids = new HashSet<>();
        List<FiducialResult> detections = frame.getFiducialResults();
        if (detections != null) for (FiducialResult tag : detections) {
            if (tag == null || HiveCell.fromTagId(tag.getFiducialId()) != selectedCell) continue;
            if (!ids.add(tag.getFiducialId())) { reject("Duplicate tag IDs"); return false; }
            if (!Double.isFinite(tag.getTargetArea()) || tag.getTargetArea() < minimumTagArea) continue;
            Pose3D tagPose = tag.getTargetPoseCameraSpace();
            if (tagPose == null || tagPose.getPosition() == null || tagPose.getPosition().unit == null
                    || tagPose.getOrientation() == null) continue;
            Position p = tagPose.getPosition().toUnit(DistanceUnit.INCH);
            double rx = tagPose.getOrientation().getRoll(AngleUnit.DEGREES);
            double ry = tagPose.getOrientation().getPitch(AngleUnit.DEGREES);
            double rz = tagPose.getOrientation().getYaw(AngleUnit.DEGREES);
            if (!finite(p.x, p.y, p.z, rx, ry, rz) || p.z <= 0) continue;
            Vec tagUp = toTurret(HiveAimGeometry.tagUp(rx, ry, rz));
            pitches.add(Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, tagUp.z)))));
            if (geometryFinite()) {
                Vec center = HiveAimGeometry.tagCenterFromTag(new Vec(p.x, p.y, p.z), rx, ry, rz, tag.getFiducialId());
                Vec opening = center.plus(HiveAimGeometry.rotate(new Vec(openingTagX, openingTagY, openingTagZ), rx, ry, rz));
                targets.add(toTurret(opening).plus(new Vec(cameraForward, cameraLeft, cameraUp)));
                Vec normal = toTurret(HiveAimGeometry.rotate(new Vec(openingNormalTagX,
                        openingNormalTagY, openingNormalTagZ), rx, ry, rz));
                normals.add(normal.scale(1 / normal.norm()));
            }
        }
        tagCount = pitches.size();
        if (tagCount < 2) { reject("Need two distinct valid tags from selected CELL"); return false; }
        pitch = 0;
        for (double value : pitches) pitch += value / tagCount;
        for (double value : pitches) if (Math.abs(value - pitch) > pitchAgreementDegrees) {
            reject("Tag orientations disagree"); return false;
        }
        state = Math.abs(pitch - upPitchDegrees) <= pitchToleranceDegrees ? CellState.UP
                : Math.abs(pitch - downPitchDegrees) <= pitchToleranceDegrees ? CellState.DOWN : CellState.TRANSITION;
        if (state != CellState.UP) {
            invalidateTarget(); status = state == CellState.DOWN ? "Selected CELL is DOWN" : "CELL tipping or orientation uncertain";
            return false;
        }
        if (!geometryFinite()) {
            invalidateTarget(); status = "Measure opening offset and nonzero outward mouth normal"; return false;
        }
        Vec target = mean(targets), normal = mean(normals);
        for (int i = 0; i < targets.size(); i++) {
            if (targets.get(i).minus(target).norm() > centerAgreementInches
                    || dot(normals.get(i), normal) < Math.cos(Math.toRadians(normalAgreementDegrees))) {
                reject("Tag-derived openings disagree; pose ambiguity or wrong geometry"); return false;
            }
        }
        double turretYaw = pose.turretDegrees + turretZeroYawDegrees;
        Vec chassis = HiveAimGeometry.rotate(target, 0, 0, turretYaw).plus(new Vec(pivotForward, pivotLeft, 0));
        Vec world = HiveAimGeometry.rotate(chassis, 0, 0, Math.toDegrees(pose.headingRadians))
                .plus(new Vec(pose.xInches, pose.yInches, 0));
        Vec worldNormal = HiveAimGeometry.rotate(normal, 0, 0, turretYaw + Math.toDegrees(pose.headingRadians));
        if (stableFrames == 0 || now - lastNewFrame < 0 || (now - lastNewFrame) * 1e-6 > maxFrameAgeMs
                || Math.abs(pitch - stablePitch) > stablePitchDegrees
                || world.minus(stabilityAnchor).norm() > stablePositionInches) {
            stableFrames = 0; stableSince = now; stabilityAnchor = world; stablePitch = pitch;
        }
        stableFrames++; lastNewFrame = now;
        targetField = world; normalField = worldNormal;
        return true;
    }

    private static Vec toTurret(Vec optical) {
        return HiveAimGeometry.opticalToTurret(optical, cameraRollDegrees, cameraPitchDegrees, cameraYawDegrees);
    }
    private static Vec mean(List<Vec> list) {
        Vec total = new Vec(0, 0, 0);
        for (Vec v : list) total = total.plus(v.scale(1.0 / list.size()));
        return total;
    }
    private static double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
    private static boolean geometryFinite() {
        return finite(openingTagX, openingTagY, openingTagZ, openingNormalTagX, openingNormalTagY, openingNormalTagZ)
                && Math.hypot(Math.hypot(openingNormalTagX, openingNormalTagY), openingNormalTagZ) > 0.01;
    }
    private static boolean validConfig() {
        return finite(cameraForward, cameraLeft, cameraUp, cameraRollDegrees, cameraPitchDegrees, cameraYawDegrees,
                pivotForward, pivotLeft, turretZeroYawDegrees, shooterForward, shooterLeft, shooterYawDegrees,
                upPitchDegrees, downPitchDegrees, pitchToleranceDegrees, pitchAgreementDegrees, centerAgreementInches,
                normalAgreementDegrees, minimumTagArea, maxFrameAgeMs, extraLatencyMs, stableDwellMs,
                stablePitchDegrees, stablePositionInches, maximumPredictionAgeMs, minimumRangeInches,
                maximumRangeInches, minimumApproachDegrees)
                && Math.abs(upPitchDegrees) <= 90 && Math.abs(downPitchDegrees) <= 90
                && pitchToleranceDegrees > 0 && Math.abs(upPitchDegrees - downPitchDegrees) > 2 * pitchToleranceDegrees
                && pitchAgreementDegrees > 0 && pitchAgreementDegrees <= 15 && centerAgreementInches > 0
                && normalAgreementDegrees > 0 && normalAgreementDegrees <= 30 && minimumTagArea > 0
                && maxFrameAgeMs > 0 && maxFrameAgeMs <= 250 && extraLatencyMs >= 0 && extraLatencyMs <= 250
                && stableDwellMs >= 100 && stablePitchDegrees > 0 && stablePitchDegrees <= 10 && stablePositionInches > 0
                && maximumPredictionAgeMs >= 0 && maximumPredictionAgeMs <= 500
                && minimumRangeInches > 0 && maximumRangeInches > minimumRangeInches
                && minimumApproachDegrees > 0 && minimumApproachDegrees <= 90;
    }
    private static double[] configSnapshot() {
        return new double[]{cameraPoseVerified ? 1 : 0, openingGeometryVerified ? 1 : 0, cellOrientationVerified ? 1 : 0,
                cameraForward, cameraLeft, cameraUp, cameraRollDegrees, cameraPitchDegrees, cameraYawDegrees,
                pivotForward, pivotLeft, turretZeroYawDegrees, shooterForward, shooterLeft, shooterYawDegrees,
                openingTagX, openingTagY, openingTagZ, openingNormalTagX, openingNormalTagY, openingNormalTagZ,
                upPitchDegrees, downPitchDegrees, pitchToleranceDegrees, pitchAgreementDegrees, centerAgreementInches,
                normalAgreementDegrees, minimumTagArea, maxFrameAgeMs, extraLatencyMs, stableDwellMs,
                stablePitchDegrees, stablePositionInches, maximumPredictionAgeMs, minimumRangeInches,
                maximumRangeInches, minimumApproachDegrees, AxonTurret.limitTravel ? 1 : 0, AxonTurret.minDegrees, AxonTurret.maxDegrees};
    }
    private void invalidateTarget() { targetField = normalField = stabilityAnchor = null; stableFrames = 0; usable = false; }
    private void invalidate() { invalidateTarget(); state = CellState.UNKNOWN; tagCount = 0; pitch = Double.NaN; }
    private void reject(String reason) { invalidate(); status = reason; }
    public void clear() { history.clear(); frames.clear(); invalidate(); lastFrame = Double.NaN; }
    /** Rechecks age and live configuration at consumption. Never interpreted as ready to fire. */
    public boolean hasAim() {
        double age = (clock.getAsLong() - captureTime) * 1e-6;
        return usable && Arrays.equals(configuration, configSnapshot()) && age >= 0
                && age <= (predicted ? maximumPredictionAgeMs : maxFrameAgeMs);
    }
    public double getTurretGoalDegrees() { return hasAim() ? goal : Double.NaN; }
    /** For calibration with motion gates false; never pass this diagnostic value to the turret. */
    public double getCandidateGoalDegrees() { return goal; }
    public double getPitchDegrees() { return pitch; }
    public double getRangeInches() { return range; }
    public double getApproachDegrees() { return approach; }
    public double getExposureAgeMs() { return ageMs; }
    public CellState getCellState() { return state; }
    public int getTagCount() { return tagCount; }
    public boolean isPredicted() { return hasAim() && predicted; }
    public String getStatus() { return usable && !hasAim() ? "Aim expired or settings changed" : status; }
}
