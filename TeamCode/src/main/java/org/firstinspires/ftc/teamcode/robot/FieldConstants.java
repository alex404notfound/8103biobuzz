package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

/**
 * One home for all field geometry, authored in the RED frame.
 * Blue values are always derived via PoseMirror.mirror(...) — never hand-written.
 *
 * KICKOFF TASK: replace every value below with the new game's field
 * (confirm the field width against Pedro's field diagram first).
 */
public class FieldConstants {
    /** Field span along X; PoseMirror reflects across FIELD_WIDTH_INCHES / 2. */
    public static final double FIELD_WIDTH_INCHES = 141.5;

    /** Where the robot sits for pre-match localization (red frame). Placeholder from DECODE. */
    public static final Pose RED_CORNER_START = new Pose(7.5, 8.1, Math.PI / 2);
}
