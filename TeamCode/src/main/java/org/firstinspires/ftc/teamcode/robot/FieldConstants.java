package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;

/**
 * BIOBUZZ nominal planning frame: audience at the bottom, red on the left,
 * origin at bottom-left, +X right, +Y up, headings counterclockwise from +X.
 * Blue equivalents rotate 180 degrees around the field center.
 * See https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09 (sections 9.1-9.3),
 * https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide (pages 8, 14-15),
 * and https://pedropathing.com/docs/pathing/reference/coordinates.
 */
public class FieldConstants {
    /** Moving CELL identities from BIOBUZZ manual 9.9 / Figure 9-17; these are not fixed poses. */
    public enum HiveCell {
        RED_FAR(Alliance.RED, 30), RED_AUDIENCE(Alliance.RED, 34),
        BLUE_AUDIENCE(Alliance.BLUE, 38), BLUE_FAR(Alliance.BLUE, 42);

        public final Alliance alliance;
        public final int firstTagId;
        HiveCell(Alliance alliance, int firstTagId) { this.alliance = alliance; this.firstTagId = firstTagId; }

        public static HiveCell fromTagId(int id) {
            for (HiveCell cell : values()) if (id >= cell.firstTagId && id < cell.firstTagId + 4) return cell;
            return null;
        }
    }

    /** Nominal dimensions, not surveyed wall distances; field construction/tile sizes vary. */
    public static final double FIELD_WIDTH_INCHES = 144.0;
    public static final double FIELD_LENGTH_INCHES = 144.0;
    public static final double FIELD_CENTER_X_INCHES = FIELD_WIDTH_INCHES / 2;
    public static final double FIELD_CENTER_Y_INCHES = FIELD_LENGTH_INCHES / 2;

    /** Set true only after measuring RED_CORNER_START and verifying its blue equivalent. */
    public static boolean START_POSE_CONFIGURED = false;

    /** Unmeasured robot-center placeholder. Field drawings do not determine a robot's start pose. */
    public static final Pose RED_CORNER_START = new Pose(7.5, 8.1, Math.PI / 2);
}
