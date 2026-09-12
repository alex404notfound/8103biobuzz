package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import java.util.List;

/** Reject incomplete or nonfinite measurements before they can drive another calibration stage. */
final class CalibrationValues {
    private CalibrationValues() { }

    static double range(double value, double min, double max, String name) {
        if (!Double.isFinite(value) || value < min || value > max)
            throw new IllegalArgumentException(name + " must be between " + min + " and " + max);
        return value;
    }

    static double positive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0)
            throw new IllegalArgumentException(name + " must be finite and positive; repeat the measurement");
        return value;
    }

    static double maximumDecelerationTarget(double forwardMaximum, double strafeMaximum) {
        positive(forwardMaximum, "Measured forward maximum velocity");
        positive(strafeMaximum, "Measured strafe maximum velocity");
        double limit = 0.8 * Math.min(forwardMaximum, strafeMaximum);
        if (limit < 1)
            throw new IllegalArgumentException("Measured forward and strafe maxima must both be at least 1.25 in/s. "
                    + "Check localization and repeat the velocity measurements before deceleration testing.");
        return limit;
    }

    static double defaultDecelerationTarget(double forwardMaximum, double strafeMaximum) {
        return Math.min(30, maximumDecelerationTarget(forwardMaximum, strafeMaximum));
    }

    static double decelerationTarget(double target, double forwardMaximum, double strafeMaximum) {
        return range(target, 1, maximumDecelerationTarget(forwardMaximum, strafeMaximum),
                "Deceleration target (in/s; limited to 80% of the slower measured maximum)");
    }

    static List<Double> coefficients(List<Double> values, int count) {
        if (values == null || values.size() != count)
            throw new IllegalArgumentException("Incomplete calibration; repeat the measurement");
        for (Double value : values)
            if (value == null || !Double.isFinite(value))
                throw new IllegalArgumentException("Nonfinite calibration; repeat the measurement");
        return values;
    }

    static List<Double> positiveCoefficients(List<Double> values, int count) {
        coefficients(values, count);
        for (double value : values) positive(value, "Feedback coefficient");
        return values;
    }
}
