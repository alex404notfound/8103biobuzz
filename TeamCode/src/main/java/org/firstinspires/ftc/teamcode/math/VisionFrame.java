package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;

/** Calibrated rigid transform from the Limelight field map into the Pedro field frame. */
public final class VisionFrame {
    private static final double INCHES_PER_METER = 100 / 2.54;
    private final double originX, originY, rotation;

    public VisionFrame(double originXInches, double originYInches, double rotationDegrees) {
        requireFinite(originXInches, originYInches, rotationDegrees);
        originX = originXInches;
        originY = originYInches;
        rotation = Math.toRadians(rotationDegrees);
    }

    public Pose toPedro(double xMeters, double yMeters, double cameraYawDegrees) {
        requireFinite(xMeters, yMeters, cameraYawDegrees);
        double x = xMeters * INCHES_PER_METER, y = yMeters * INCHES_PER_METER;
        return new Pose(originX + x * Math.cos(rotation) - y * Math.sin(rotation),
                originY + x * Math.sin(rotation) + y * Math.cos(rotation),
                Math.toRadians(cameraYawDegrees) + rotation);
    }

    /** Inverse heading transform: MegaTag2 expects its own field-map frame. */
    public double toCameraHeading(double pedroHeadingRadians) {
        requireFinite(pedroHeadingRadians);
        double angle = pedroHeadingRadians - rotation;
        return Math.toDegrees(Math.atan2(Math.sin(angle), Math.cos(angle)));
    }

    private static void requireFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) throw new IllegalArgumentException("Vision frame values must be finite");
        }
    }
}
