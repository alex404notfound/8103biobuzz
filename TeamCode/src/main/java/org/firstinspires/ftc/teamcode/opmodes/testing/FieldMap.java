package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

/**
 * Open Panels at http://192.168.43.1:8001 and add the Field widget.
 * In INIT, enter the measured robot-center pose under FieldMap, then press A.
 * After START, move the robot by hand; this diagnostic never commands driving.
 */
@Configurable
@Disabled
@NextTeleop(name = "Field Map", group = "Diagnostics")
public class FieldMap extends RobotOpMode {
    // Editable proposals only: INIT + A explicitly applies them to localization.
    public static double startX = 72;
    public static double startY = 24;
    public static double startHeadingDegrees = 90;
    private String poseStatus = "Using previous pose; enter measured start and press A in INIT";

    public FieldMap(Robot robot) { super(robot); }

    @Override protected void onInit() {
        robot.drivetrain.usePreviousStartingPose();
        showInstructions();
    }

    @Override protected void onInitLoop() {
        if (gamepad1.xWasPressed()) {
            robot.drivetrain.recalibrateLocalization();
            poseStatus = "Keep still until Pinpoint is READY, then press A to apply measured start";
        } else if (gamepad1.aWasPressed()) {
            double x = startX, y = startY, heading = startHeadingDegrees;
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)
                    || x < 0 || x > FieldConstants.FIELD_WIDTH_INCHES
                    || y < 0 || y > FieldConstants.FIELD_LENGTH_INCHES) {
                poseStatus = "Start rejected: X/Y must be within 0..144 inches; heading must be finite";
            } else if (!robot.drivetrain.isLocalizationReady()) {
                poseStatus = "Wait for Pinpoint READY, then press A again";
            } else {
                robot.drivetrain.setStartingPose(new Pose(x, y, Math.toRadians(heading)));
                poseStatus = "Measured starting pose applied; START, then move robot by hand";
            }
        }
        showInstructions();
    }

    @Override protected void onLoop() {
        robot.drivetrain.stop();
        showInstructions();
    }

    private void showInstructions() {
        robot.telemetry.addLine("Field map: Panels http://192.168.43.1:8001 > Field");
        robot.telemetry.addLine("INIT: X calibrates gyro; A applies FieldMap startX/startY/startHeadingDegrees.");
        robot.telemetry.addLine("Origin bottom-left, audience below. 0 degrees right; 90 degrees away from audience.");
        robot.telemetry.addLine("Move by hand after START. Skeleton TeleOp also shows the map while driving.");
        robot.telemetry.addData("Field starting pose", poseStatus);
    }
}
