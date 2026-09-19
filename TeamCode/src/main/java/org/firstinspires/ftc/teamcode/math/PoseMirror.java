package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.FieldConstants;

public class PoseMirror {
    /** BIOBUZZ alliance transform: rotate 180 degrees around the nominal field center. */
    public static Pose mirror(Pose pose) {
        return new Pose(
                FieldConstants.FIELD_WIDTH_INCHES - pose.x(),
                FieldConstants.FIELD_LENGTH_INCHES - pose.y(),
                mirrorHeading(pose.heading())
        );
    }

    /** Rotate a heading by 180 degrees, returning radians in [0, 2 pi). */
    public static double mirrorHeading(double headingRadians) {
        double heading = (headingRadians + Math.PI) % (2 * Math.PI);
        return heading < 0 ? heading + 2 * Math.PI : heading;
    }
}
