package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.math.HiveAimGeometry;
import org.firstinspires.ftc.teamcode.math.HiveAimGeometry.Vec;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;

import java.util.Arrays;
import java.util.function.LongSupplier;

/** Pinpoint field-pose azimuth aiming. The driver selects the raised CELL; no camera is used. */
@Configurable
public class HiveAutoAim {
    // Nominal raised-opening coordinates. Adjust to a measured practice field in the Pedro frame.
    public static volatile double redHiveX = FieldConstants.RED_HIVE_X_INCHES;
    public static volatile double blueHiveX = FieldConstants.BLUE_HIVE_X_INCHES;
    public static volatile double audienceCellY = FieldConstants.AUDIENCE_CELL_UP_Y_INCHES;
    public static volatile double farCellY = FieldConstants.FAR_CELL_UP_Y_INCHES;
    // Inches in chassis forward/left axes. Zero offsets mean a centered, forward-facing turret.
    public static volatile double pivotForward = 0, pivotLeft = 0, turretZeroYawDegrees = 0;
    // Muzzle position and bore yaw in the rotating turret frame.
    public static volatile double shooterForward = 0, shooterLeft = 0, shooterYawDegrees = 0;
    public static volatile double minimumRangeInches = 12, maximumRangeInches = 180;
    /** Horizontal approach: 90 = in front of the mouth, 0 = alongside, negative = behind. */
    public static volatile double minimumApproachDegrees = 30;
    public static volatile double maximumPoseAgeMs = 100;

    private final LongSupplier clock;
    private double[] configuration;
    private long sampleTime;
    private boolean usable;
    private double goal = Double.NaN, range = Double.NaN, approach = Double.NaN;
    private double targetX = Double.NaN, targetY = Double.NaN, fieldBearing = Double.NaN;
    private String status = "Apply a measured field pose and calibrate the turret servo";

    public HiveAutoAim() { this(System::nanoTime); }
    HiveAutoAim(LongSupplier clock) { this.clock = clock; }

    /** Call after a fresh Pinpoint sample. Heading is radians; turret angles are CCW-positive degrees. */
    public void update(boolean fieldPoseInitialized, boolean healthy, double x, double y,
                       double headingRadians, double turretDegrees, double minDegrees, double maxDegrees,
                       Alliance alliance, HiveCell cell) {
        resetSolution();
        sampleTime = clock.getAsLong();
        configuration = configSnapshot();
        if (!fieldPoseInitialized) { status = "Apply a measured field pose in INIT"; return; }
        if (!healthy || !finite(x, y, headingRadians)) {
            status = "Healthy Pinpoint pose required"; return;
        }
        if (!finite(turretDegrees, minDegrees, maxDegrees) || minDegrees >= maxDegrees) {
            status = "Set the servo sweep, zero and safe positions before INIT"; return;
        }
        if (cell == null || alliance == null || cell.alliance != alliance) {
            status = "Select a CELL on your alliance's HIVE"; return;
        }
        if (!validConfig()) { status = "Invalid pose-aim settings"; return; }
        if (!insideField(x, y)) { status = "Robot pose is outside the configured field frame"; return; }

        targetX = alliance == Alliance.RED ? redHiveX : blueHiveX;
        targetY = cell.audienceSide ? audienceCellY : farCellY;
        double headingDegrees = Math.toDegrees(headingRadians);
        Vec pivot = new Vec(x, y, 0).plus(HiveAimGeometry.rotate(new Vec(pivotForward, pivotLeft, 0),
                0, 0, headingDegrees));
        Vec target = new Vec(targetX, targetY, 0);
        Vec relative = HiveAimGeometry.rotate(target.minus(pivot), 0, 0, -headingDegrees);
        fieldBearing = Math.toDegrees(Math.atan2(target.y - pivot.y, target.x - pivot.x));
        double bearing = HiveAimGeometry.turretBearingDegrees(relative, shooterForward, shooterLeft, shooterYawDegrees);
        goal = HiveAimGeometry.nearestTurretDegrees(bearing - turretZeroYawDegrees, turretDegrees,
                true, minDegrees, maxDegrees);
        if (!finite(bearing, goal)) { status = "No forward muzzle solution inside turret travel"; return; }

        Vec muzzle = pivot.plus(HiveAimGeometry.rotate(new Vec(shooterForward, shooterLeft, 0),
                0, 0, headingDegrees + bearing));
        Vec towardShooter = muzzle.minus(target);
        range = Math.hypot(towardShooter.x, towardShooter.y);
        double outwardY = cell.audienceSide ? -1 : 1;
        double cosine = towardShooter.y * outwardY / range;
        approach = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, cosine))));
        if (!finite(range, approach) || range < minimumRangeInches || range > maximumRangeInches
                || approach < minimumApproachDegrees) {
            status = "Outside range or approach too shallow/behind selected opening"; return;
        }
        usable = true;
        status = "Pinpoint azimuth available; driver selects raised CELL";
    }

    private static boolean insideField(double x, double y) {
        return x >= 0 && x <= FieldConstants.FIELD_WIDTH_INCHES
                && y >= 0 && y <= FieldConstants.FIELD_LENGTH_INCHES;
    }
    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }
    private static boolean validConfig() {
        return finite(redHiveX, blueHiveX, audienceCellY, farCellY,
                pivotForward, pivotLeft, turretZeroYawDegrees, shooterForward, shooterLeft, shooterYawDegrees,
                minimumRangeInches, maximumRangeInches, minimumApproachDegrees, maximumPoseAgeMs)
                && insideField(redHiveX, audienceCellY) && insideField(blueHiveX, farCellY)
                && redHiveX < blueHiveX && audienceCellY < farCellY
                && minimumRangeInches > 0 && maximumRangeInches > minimumRangeInches
                && minimumApproachDegrees > 0 && minimumApproachDegrees <= 90
                && maximumPoseAgeMs > 0 && maximumPoseAgeMs <= 250;
    }
    private static double[] configSnapshot() {
        return new double[]{redHiveX, blueHiveX, audienceCellY, farCellY,
                pivotForward, pivotLeft, turretZeroYawDegrees, shooterForward, shooterLeft, shooterYawDegrees,
                minimumRangeInches, maximumRangeInches, minimumApproachDegrees, maximumPoseAgeMs};
    }
    private void resetSolution() {
        usable = false;
        goal = range = approach = targetX = targetY = fieldBearing = Double.NaN;
    }
    public void clear() { resetSolution(); configuration = null; status = "Stopped"; }
    /** A fresh horizontal pointing solution, never a firing permission or observed CELL state. */
    public boolean hasAim() {
        double ageMs = (clock.getAsLong() - sampleTime) * 1e-6;
        return usable && Arrays.equals(configuration, configSnapshot()) && ageMs >= 0 && ageMs <= maximumPoseAgeMs;
    }
    public double getTurretGoalDegrees() { return hasAim() ? goal : Double.NaN; }
    public double getCandidateGoalDegrees() { return goal; }
    public double getFieldBearingDegrees() { return fieldBearing; }
    public double getTargetXInches() { return targetX; }
    public double getTargetYInches() { return targetY; }
    /** Horizontal distance from the offset muzzle to the mapped aperture point. */
    public double getRangeInches() { return range; }
    public double getApproachDegrees() { return approach; }
    public String getStatus() { return usable && !hasAim() ? "Pose sample expired or settings changed" : status; }
}
