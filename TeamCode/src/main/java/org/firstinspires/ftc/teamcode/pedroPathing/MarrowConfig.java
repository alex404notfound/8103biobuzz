package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;
import com.skeletonarmy.marrow.zones.Zone;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;

import java.util.ArrayList;
import java.util.List;

/** Measured field-frame setup for Hive Marrow Test; captured on INIT, never edited during motion. */
@Configurable
public final class MarrowConfig {
    private MarrowConfig() { }

    public static boolean geometryAndMapVerified = false;
    public static double robotLengthInches = 18;
    public static double robotWidthInches = 18;
    public static double clearanceInches = 4;
    public static double startX = 72;
    public static double startY = 24;
    public static double startHeadingDegrees = 90;
    // Optional measured practice obstacle. This is not a map of the BIOBUZZ HIVE.
    public static boolean obstacleEnabled = false;
    public static double obstacleX = 72;
    public static double obstacleY = 72;
    public static double obstacleRadiusInches = 8;

    public static MarrowPlanner createPlanner() {
        if (!geometryAndMapVerified)
            throw new IllegalArgumentException("Measure MarrowConfig footprint, starting pose and keep-out map; then set geometryAndMapVerified");
        List<Zone> obstacles = new ArrayList<>();
        if (obstacleEnabled) {
            if (!Double.isFinite(obstacleX) || !Double.isFinite(obstacleY)
                    || !Double.isFinite(obstacleRadiusInches) || obstacleRadiusInches <= 0)
                throw new IllegalArgumentException("Marrow practice obstacle must have a finite center and positive radius");
            obstacles.add(new CircleZone(new Point(obstacleX, obstacleY), obstacleRadiusInches));
        }
        return new MarrowPlanner(robotLengthInches, robotWidthInches, clearanceInches,
                new PolygonZone(new Point(FieldConstants.FIELD_CENTER_X_INCHES, FieldConstants.FIELD_CENTER_Y_INCHES),
                        FieldConstants.FIELD_WIDTH_INCHES, FieldConstants.FIELD_LENGTH_INCHES), obstacles);
    }

    public static Pose startingPose() {
        return new Pose(startX, startY, Math.toRadians(startHeadingDegrees));
    }
}
