package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;

import java.util.List;
import java.util.function.LongSupplier;

/** Selected-tag alignment only. A stationary/visible tag does not prove its CELL is upward. */
@Config
public class HiveTagAim {
    public static double maxFrameAgeMs = 100, minimumArea = 0.02;
    public static double targetTxDegrees = 0, turnKp = 0.015, maximumTurnPower = 0.18;
    /** Positive tx is camera-right. Reverse this if the mounted camera/drive test requires it. */
    public static double turnSign = 1;
    public static double toleranceDegrees = 1, alignedDwellMs = 200, verticalStabilityDegrees = 1.5;

    private final LongSupplier clock;
    private final VisionResultGate frames;
    private boolean visible;
    private int selectedId = -1, alignedFrames;
    private Alliance selectedAlliance;
    private double setpoint = Double.NaN, lastTimestamp = Double.NaN;
    private double tx, ty, error, rangeInches = Double.NaN, stableTy;
    private long alignedSince, lastNewFrame;
    private String status = "No selected tag";

    public HiveTagAim(int pipeline) { this(pipeline, System::nanoTime); }
    HiveTagAim(int pipeline, LongSupplier clock) {
        this.clock = clock;
        frames = new VisionResultGate(pipeline, clock);
    }

    public void update(LLResult received, Alliance alliance, int tagId) {
        frames.observe(received);
        if (selectedId != tagId || selectedAlliance != alliance || setpoint != targetTxDegrees) {
            resetAlignment();
            lastTimestamp = Double.NaN;
            selectedId = tagId;
            selectedAlliance = alliance;
            setpoint = targetTxDegrees;
        }
        HiveCell cell = HiveCell.fromTagId(tagId);
        if (alliance == null || cell == null || cell.alliance != alliance) {
            reject("Select a tag on your alliance's hive"); return;
        }
        if (!validConfig()) { reject("Invalid aiming configuration"); return; }
        LLResult fresh = frames.getFresh(maxFrameAgeMs);
        if (fresh == null) { reject("No fresh AprilTag frame from selected pipeline"); return; }
        FiducialResult target = find(fresh.getFiducialResults(), tagId);
        if (target == null) { reject("Selected tag not visible; no substitution"); return; }
        tx = target.getTargetXDegrees();
        ty = target.getTargetYDegrees();
        if (!Double.isFinite(tx) || !Double.isFinite(ty) || Math.abs(tx) >= 90 || Math.abs(ty) >= 90
                || !Double.isFinite(target.getTargetArea()) || target.getTargetArea() < minimumArea) {
            reject("Invalid or too-small selected tag"); return;
        }
        visible = true;
        error = tx - targetTxDegrees;
        rangeInches = range(target.getTargetPoseCameraSpace());
        long now = clock.getAsLong();
        if (Double.compare(lastTimestamp, fresh.getTimestamp()) != 0) {
            if (fresh.getTimestamp() < lastTimestamp || now < lastNewFrame
                    || (now - lastNewFrame) / 1e6 > maxFrameAgeMs) resetAlignment();
            lastTimestamp = fresh.getTimestamp();
            lastNewFrame = now;
            if (Math.abs(error) > toleranceDegrees) resetAlignment();
            else {
                if (alignedFrames == 0 || Math.abs(ty - stableTy) > verticalStabilityDegrees) {
                    alignedSince = now;
                    stableTy = ty;
                    alignedFrames = 0;
                }
                alignedFrames++;
            }
        }
        status = isAligned() ? "TAG ALIGNED - driver must verify upward CELL" : "Tracking selected tag";
    }

    public boolean hasTarget() { return visible && validConfig() && frames.getFresh(maxFrameAgeMs) != null; }

    public double getTurnPower() {
        if (!hasTarget() || Math.abs(error) <= toleranceDegrees) return 0;
        return Math.max(-maximumTurnPower, Math.min(maximumTurnPower, turnSign * turnKp * error));
    }

    public boolean isAligned() {
        return hasTarget() && alignedFrames >= 3 && Math.abs(error) <= toleranceDegrees
                && (lastNewFrame - alignedSince) / 1e6 >= alignedDwellMs;
    }

    public double getTx() { return hasTarget() ? tx : Double.NaN; }
    public double getTy() { return hasTarget() ? ty : Double.NaN; }
    public double getErrorDegrees() { return hasTarget() ? error : Double.NaN; }
    /** Camera-to-tag slant distance, not horizontal launcher-to-opening distance. */
    public double getRangeInches() { return hasTarget() ? rangeInches : Double.NaN; }
    public String getStatus() { return visible && !hasTarget() ? "Selected tag expired" : status; }

    public void clear() {
        frames.clear();
        reject("Stopped");
    }

    private void reject(String reason) { visible = false; resetAlignment(); status = reason; }
    private void resetAlignment() { alignedFrames = 0; }

    private static FiducialResult find(List<FiducialResult> detections, int id) {
        if (detections != null) for (FiducialResult detection : detections)
            if (detection != null && detection.getFiducialId() == id) return detection;
        return null;
    }

    private static double range(Pose3D pose) {
        if (pose == null || pose.getPosition() == null || pose.getPosition().unit == null) return Double.NaN;
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        double distance = Math.hypot(Math.hypot(p.x, p.y), p.z);
        // SDK defaults missing 3D poses to zero. Do not display that as a measured range.
        return Double.isFinite(distance) && distance > 0 ? distance : Double.NaN;
    }

    private boolean validConfig() {
        return Double.isFinite(maxFrameAgeMs) && maxFrameAgeMs > 0 && maxFrameAgeMs <= 250
                && Double.isFinite(minimumArea) && minimumArea > 0
                && Double.isFinite(targetTxDegrees) && Math.abs(targetTxDegrees) < 90
                && setpoint == targetTxDegrees
                && Double.isFinite(turnKp) && turnKp > 0 && turnKp <= 1
                && Double.isFinite(maximumTurnPower) && maximumTurnPower >= 0 && maximumTurnPower <= 0.35
                && (turnSign == 1 || turnSign == -1)
                && Double.isFinite(toleranceDegrees) && toleranceDegrees > 0 && toleranceDegrees <= 10
                && Double.isFinite(alignedDwellMs) && alignedDwellMs >= 0
                && Double.isFinite(verticalStabilityDegrees) && verticalStabilityDegrees > 0;
    }
}
