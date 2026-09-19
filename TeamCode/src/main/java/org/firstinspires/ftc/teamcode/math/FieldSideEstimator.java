package org.firstinspires.ftc.teamcode.math;

import org.firstinspires.ftc.teamcode.robot.FieldConstants;

/**
 * Describes position in the nominal audience-bottom/red-left field frame.
 * Pinpoint needs an explicitly initialized field pose; relative odometry alone has no field side.
 * This is a position label and must never select or change the robot's scoring alliance.
 */
public final class FieldSideEstimator {
    public enum Lateral { UNKNOWN, RED_HALF, CENTER_BAND, BLUE_HALF }
    public enum Longitudinal { UNKNOWN, AUDIENCE_HALF, CENTER_BAND, FAR_HALF }

    public static final class Result {
        public final Lateral lateral;
        public final Longitudinal longitudinal;

        private Result(Lateral lateral, Longitudinal longitudinal) {
            this.lateral = lateral;
            this.longitudinal = longitudinal;
        }

        public boolean isKnown() { return lateral != Lateral.UNKNOWN; }
    }

    private static final Result UNKNOWN = new Result(Lateral.UNKNOWN, Longitudinal.UNKNOWN);

    private FieldSideEstimator() { }

    /**
     * uncertaintyInches is a manually chosen bound, not a measured Pinpoint confidence estimate.
     * A coordinate whose uncertainty reaches the center line is labeled CENTER_BAND.
     */
    public static Result estimate(double xInches, double yInches,
                                  boolean fieldPoseInitialized, boolean odometryHealthy,
                                  double uncertaintyInches) {
        if (!fieldPoseInitialized || !odometryHealthy
                || !Double.isFinite(xInches) || !Double.isFinite(yInches)
                || !Double.isFinite(uncertaintyInches) || uncertaintyInches < 0
                || uncertaintyInches >= Math.min(FieldConstants.FIELD_CENTER_X_INCHES,
                        FieldConstants.FIELD_CENTER_Y_INCHES)
                || xInches < 0 || xInches > FieldConstants.FIELD_WIDTH_INCHES
                || yInches < 0 || yInches > FieldConstants.FIELD_LENGTH_INCHES) return UNKNOWN;
        double dx = xInches - FieldConstants.FIELD_CENTER_X_INCHES;
        double dy = yInches - FieldConstants.FIELD_CENTER_Y_INCHES;
        Lateral lateral = Math.abs(dx) <= uncertaintyInches ? Lateral.CENTER_BAND
                : dx < 0 ? Lateral.RED_HALF : Lateral.BLUE_HALF;
        Longitudinal longitudinal = Math.abs(dy) <= uncertaintyInches ? Longitudinal.CENTER_BAND
                : dy < 0 ? Longitudinal.AUDIENCE_HALF : Longitudinal.FAR_HALF;
        return new Result(lateral, longitudinal);
    }
}
