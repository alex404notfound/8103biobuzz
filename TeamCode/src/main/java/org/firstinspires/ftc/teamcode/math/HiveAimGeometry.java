package org.firstinspires.ftc.teamcode.math;

/**
 * Geometry for individual BIOBUZZ tags, independent of a fixed AprilTag field map.
 * Distances are inches and angles are degrees. Camera optical axes are right/down/forward;
 * turret axes are forward/left/up. SDK12 stores optical rx/ry/rz as roll/pitch/yaw unchanged.
 */
public final class HiveAimGeometry {
    // FIRST BIOBUZZ manual TU01, Figure 9-15: viewed with the cluster sticker upright.
    private static final double[] TAG_ROW_OFFSETS_INCHES = {-6.5, -2.75, 2.75, 6.5};

    private HiveAimGeometry() { }

    public static final class Vec {
        public final double x, y, z;

        public Vec(double x, double y, double z) {
            requireFinite(x, y, z);
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vec plus(Vec other) { return new Vec(x + other.x, y + other.y, z + other.z); }
        public Vec minus(Vec other) { return new Vec(x - other.x, y - other.y, z - other.z); }
        public Vec scale(double factor) { return new Vec(x * factor, y * factor, z * factor); }
        public double norm() { return Math.hypot(Math.hypot(x, y), z); }
    }

    /** Extrinsic right-handed X, then Y, then Z rotations: Rz(yaw) Ry(pitch) Rx(roll). */
    public static Vec rotate(Vec v, double rollDeg, double pitchDeg, double yawDeg) {
        requireFinite(rollDeg, pitchDeg, yawDeg);
        double roll = Math.toRadians(rollDeg), pitch = Math.toRadians(pitchDeg);
        double yaw = Math.toRadians(yawDeg);
        double cr = Math.cos(roll), sr = Math.sin(roll);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        double x1 = v.x, y1 = cr * v.y - sr * v.z, z1 = sr * v.y + cr * v.z;
        double x2 = cp * x1 + sp * z1, y2 = y1, z2 = -sp * x1 + cp * z1;
        return new Vec(cy * x2 - sy * y2, sy * x2 + cy * y2, z2);
    }

    /**
     * Converts an optical vector to turret forward/left/up, then applies camera mounting rotation.
     * Positive mounting pitch tilts camera forward DOWN; positive yaw turns it left.
     * For points, add the measured camera translation separately after this rotation.
     */
    public static Vec opticalToTurret(Vec cameraVector, double rollDeg,
                                      double pitchDeg, double yawDeg) {
        return rotate(new Vec(cameraVector.z, -cameraVector.x, -cameraVector.y),
                rollDeg, pitchDeg, yawDeg);
    }

    /**
     * Returns the tag-row center in camera optical space, not the opening center.
     * Input translation must already be converted from SDK meters to inches. The measured
     * opening offset must subsequently be transformed and added in the same tag coordinate frame.
     */
    public static Vec tagCenterFromTag(Vec opticalPosition, double rx, double ry,
                                       double rz, int tagId) {
        if (tagId < 30 || tagId > 45) throw new IllegalArgumentException("Expected a BIOBUZZ tag ID 30-45");
        Vec tagOffset = new Vec(TAG_ROW_OFFSETS_INCHES[(tagId - 30) % 4], 0, 0);
        return opticalPosition.minus(rotate(tagOffset, rx, ry, rz));
    }

    /** Unit vector toward the printed tag's top edge, in camera optical space. */
    public static Vec tagUp(double rx, double ry, double rz) {
        return rotate(new Vec(0, -1, 0), rx, ry, rz);
    }

    /** Unit normal to the tag plane, in camera optical space; do not assume its sign is gravity-up. */
    public static Vec tagNormal(double rx, double ry, double rz) {
        return rotate(new Vec(0, 0, 1), rx, ry, rz);
    }

    /**
     * Solves turret yaw for an offset muzzle and bore direction, all relative to the turret pivot.
     * The target is in chassis forward/left/up coordinates. The muzzle offsets and shooter yaw
     * are in the rotating turret frame. Height is intentionally ignored: this solves horizontal
     * pointing only, not a launch trajectory. Returns NaN for a tangent or backward intersection.
     */
    public static double turretBearingDegrees(Vec targetFromPivot, double shooterForward,
                                               double shooterLeft, double shooterYawDeg) {
        if (!Double.isFinite(shooterForward) || !Double.isFinite(shooterLeft)
                || !Double.isFinite(shooterYawDeg)) return Double.NaN;
        double distance = Math.hypot(targetFromPivot.x, targetFromPivot.y);
        if (!Double.isFinite(distance) || distance <= 0) return Double.NaN;
        double yaw = Math.toRadians(shooterYawDeg);
        double sinYaw = Math.sin(yaw), cosYaw = Math.cos(yaw);
        double ratio = (shooterForward * sinYaw - shooterLeft * cosYaw) / distance;
        if (!Double.isFinite(ratio) || Math.abs(ratio) >= 1) return Double.NaN;
        double forwardAlongRay = distance * Math.sqrt(1 - ratio * ratio)
                - shooterForward * cosYaw - shooterLeft * sinYaw;
        if (!Double.isFinite(forwardAlongRay) || forwardAlongRay <= 0) return Double.NaN;
        double bearing = Math.atan2(targetFromPivot.y, targetFromPivot.x) - yaw + Math.asin(ratio);
        return Math.toDegrees(Math.atan2(Math.sin(bearing), Math.cos(bearing)));
    }

    /**
     * Chooses the equivalent bearing closest to the turret's unwrapped current angle.
     * A limited turret may use a longer route to stay within its physical interval. Returns NaN
     * when no equivalent fits, or when a required input is invalid; never clamps an unreachable
     * bearing to a travel endpoint and misrepresents that endpoint as an aiming solution.
     */
    public static double nearestTurretDegrees(double bearingDeg, double currentDeg,
                                              boolean limited, double min, double max) {
        if (!Double.isFinite(bearingDeg) || !Double.isFinite(currentDeg)) return Double.NaN;
        double bearing = Math.IEEEremainder(bearingDeg, 360);
        if (!limited) {
            double delta = Math.IEEEremainder(bearing - Math.IEEEremainder(currentDeg, 360), 360);
            double result = currentDeg + delta;
            return Double.isFinite(result) ? result : Double.NaN;
        }
        if (!Double.isFinite(min) || !Double.isFinite(max) || min > max) return Double.NaN;
        double firstTurn = Math.ceil((min - bearing) / 360);
        double lastTurn = Math.floor((max - bearing) / 360);
        if (firstTurn > lastTurn) return Double.NaN;
        double nearestTurn = Math.rint((currentDeg - bearing) / 360);
        double turn = Math.max(firstTurn, Math.min(lastTurn, nearestTurn));
        double result = bearing + 360 * turn;
        return Double.isFinite(result) && result >= min && result <= max ? result : Double.NaN;
    }

    private static void requireFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) throw new IllegalArgumentException("Geometry values must be finite");
        }
    }
}
