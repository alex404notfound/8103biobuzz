package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.FieldConstants;

public class PoseMirror {
    /** Reflect across the field centerline: x -> width - x, heading -> pi - heading, y unchanged. */
    public static Pose mirror(Pose pose) {
        return new Pose(
                FieldConstants.FIELD_WIDTH_INCHES - pose.x(),
                pose.y(),
                Math.PI - pose.heading()
        );
    }
}
