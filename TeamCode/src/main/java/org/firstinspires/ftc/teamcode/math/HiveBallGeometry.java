package org.firstinspires.ftc.teamcode.math;

/**
 * Floor-ball projection adapted from Hive Vision's BallMath.camRay (MIT, Harjas Sidhu).
 * See third_party/hive-vision/LICENSE and docs/HIVE_VISION_TEST.md for provenance.
 */
public final class HiveBallGeometry {
    public static final class Position {
        public final double forwardInches, leftInches, cameraRangeInches;

        private Position(double forward, double left, double range) {
            forwardInches = forward;
            leftInches = left;
            cameraRangeInches = range;
        }
    }

    private final double heightDifference, pitch, yaw, forwardOffset, leftOffset, maximumRange;

    /** Measured, fixed, zero-roll camera mount. Heights/offsets/range are inches. */
    public HiveBallGeometry(double cameraHeight, double ballCenterHeight, double pitchDownDegrees,
                            double yawLeftDegrees, double forwardOffset, double leftOffset,
                            double maximumRange) {
        for (double value : new double[]{cameraHeight, ballCenterHeight, pitchDownDegrees,
                yawLeftDegrees, forwardOffset, leftOffset, maximumRange}) {
            if (!Double.isFinite(value)) throw new IllegalArgumentException("Camera measurements must be finite");
        }
        if (ballCenterHeight <= 0 || cameraHeight <= ballCenterHeight || maximumRange <= 0
                || pitchDownDegrees < 0 || pitchDownDegrees >= 90) {
            throw new IllegalArgumentException("Measure camera above ball center, pitch in [0,90), and positive range");
        }
        heightDifference = cameraHeight - ballCenterHeight;
        pitch = Math.toRadians(pitchDownDegrees);
        yaw = Math.toRadians(yawLeftDegrees);
        this.forwardOffset = forwardOffset;
        this.leftOffset = leftOffset;
        this.maximumRange = maximumRange;
    }

    /** Optical-center angles: tx is right-positive, ty up-positive. Null means no usable floor intersection. */
    public Position project(double txDegrees, double tyDegrees) {
        if (!Double.isFinite(txDegrees) || !Double.isFinite(tyDegrees)
                || Math.abs(txDegrees) >= 89 || Math.abs(tyDegrees) >= 89) return null;
        double horizontal = Math.tan(Math.toRadians(txDegrees));
        double vertical = Math.tan(Math.toRadians(tyDegrees));
        // Unnormalized pinhole ray, rotated down about camera-left. Its scale cancels below.
        double forward = Math.cos(pitch) + vertical * Math.sin(pitch);
        double left = -horizontal;
        double up = -Math.sin(pitch) + vertical * Math.cos(pitch);
        if (up >= -1e-6 || forward <= 0) return null;
        double scale = heightDifference / -up;
        forward *= scale;
        left *= scale;
        double range = Math.hypot(forward, left);
        if (!Double.isFinite(range) || range > maximumRange) return null;
        return new Position(forwardOffset + forward * Math.cos(yaw) - left * Math.sin(yaw),
                leftOffset + forward * Math.sin(yaw) + left * Math.cos(yaw), range);
    }
}
