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
    /** CELL identity; the driver selects the raised end when using pose-only aiming. */
    public enum HiveCell {
        RED_FAR(Alliance.RED, 30, false), RED_AUDIENCE(Alliance.RED, 34, true),
        BLUE_AUDIENCE(Alliance.BLUE, 38, true), BLUE_FAR(Alliance.BLUE, 42, false);

        public final Alliance alliance;
        public final int firstTagId;
        public final boolean audienceSide;
        HiveCell(Alliance alliance, int firstTagId, boolean audienceSide) {
            this.alliance = alliance; this.firstTagId = firstTagId; this.audienceSide = audienceSide;
        }

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

    /**
     * Nominal UP opening map, in the same Pedro frame as Pinpoint. FIRST manual TU04,
     * Figures 9-8 through 9-11: HIVE centers 25.5 inches apart, stable tilt 30 degrees.
     * Official STEP v26-27.2: the raised mouth's bottom edge is about 19.3 inches
     * from the centerline. A point halfway up the 14-inch mouth projects inward
     * by 7*sin(30)=3.5 inches, leaving a nominal 15.8-inch horizontal offset.
     * This chosen aperture point is for azimuth, not an official ballistic aim point.
     * https://ftc-resources.firstinspires.org/ftc/game/manual-09
     * https://ftc-resources.firstinspires.org/ftc/field/field-cad-step
     */
    public static final double HIVE_CENTER_SPACING_INCHES = 25.5;
    public static final double HIVE_STABLE_TILT_DEGREES = 30;
    public static final double RAISED_OPENING_OFFSET_INCHES = 15.8;
    public static final double RED_HIVE_X_INCHES = FIELD_CENTER_X_INCHES - HIVE_CENTER_SPACING_INCHES / 2;
    public static final double BLUE_HIVE_X_INCHES = FIELD_CENTER_X_INCHES + HIVE_CENTER_SPACING_INCHES / 2;
    public static final double AUDIENCE_CELL_UP_Y_INCHES = FIELD_CENTER_Y_INCHES - RAISED_OPENING_OFFSET_INCHES;
    public static final double FAR_CELL_UP_Y_INCHES = FIELD_CENTER_Y_INCHES + RAISED_OPENING_OFFSET_INCHES;

    /** Set true only after measuring RED_CORNER_START and verifying its blue equivalent. */
    public static boolean START_POSE_CONFIGURED = false;

    /** Unmeasured robot-center placeholder. Field drawings do not determine a robot's start pose. */
    public static final Pose RED_CORNER_START = new Pose(7.5, 8.1, Math.PI / 2);
}
