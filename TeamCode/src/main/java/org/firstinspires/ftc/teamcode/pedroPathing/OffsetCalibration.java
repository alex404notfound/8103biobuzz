package org.firstinspires.ftc.teamcode.pedroPathing;

/** Pure offset-tuning math, kept separate from the FTC hardware lifecycle for host-side tests. */
final class OffsetCalibration {
    private static final double HALF_TURN_RADIANS = Math.PI;

    private OffsetCalibration() {}

    static boolean hasCompletedHalfTurn(double totalHeading) {
        return !Double.isNaN(totalHeading)
                && !Double.isInfinite(totalHeading)
                && Math.abs(totalHeading) >= HALF_TURN_RADIANS;
    }

    static Result calculate(double poseX, double poseY, double totalHeading) {
        double sin = Math.sin(totalHeading);
        double oneMinusCos = 1 - Math.cos(totalHeading);
        double divisor = sin * sin + oneMinusCos * oneMinusCos;

        if (Double.isNaN(divisor) || Double.isInfinite(divisor) || divisor < 1e-9) {
            throw new IllegalArgumentException("A nonzero rotation is required to calculate offsets");
        }

        // Pinpoint pose X maps to Pedro forward and pose Y maps to Pedro left. These equations
        // invert the displacement produced by rotating with device compensation set to zero.
        double strafePodX = (-oneMinusCos * poseX + sin * poseY) / divisor;
        double forwardPodY = (-sin * poseX - oneMinusCos * poseY) / divisor;
        return new Result(strafePodX, forwardPodY);
    }

    static final class Result {
        final double strafePodX;
        final double forwardPodY;

        Result(double strafePodX, double forwardPodY) {
            this.strafePodX = strafePodX;
            this.forwardPodY = forwardPodY;
        }
    }
}
