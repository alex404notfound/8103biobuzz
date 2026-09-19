package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

/**
 * Drive-only starting point for the season's competition teleop.
 * Override onInit()/onLoop(); the base class owns cache clearing, the scheduler,
 * subsystem updates, and cleanup. Bindings use the SDK's edge-detection
 * methods (xWasPressed()/xWasReleased()), never hand-rolled previous-state booleans.
 * gamepad1 = driving + primary mechanism triggers; gamepad2 = operator overrides.
 */
@TeleOp(name = "Skeleton TeleOp", group = "Competition")
public class SkeletonTeleOp extends RobotOpMode {

    @Override
    protected void onInit() {
        robot.drivetrain.usePreviousStartingPose(); // carried over from auto / corner localizer
    }

    @Override
    protected void onLoop() {
        // Turning releases any heading lock.
        if (Math.abs(gamepad1.right_stick_x) >= 0.1) robot.drivetrain.unlockHeading();

        robot.drivetrain.arcadeDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                Alliance.current
        );

        // Live example bindings, showing the edge-detection convention:
        if (gamepad1.triangleWasPressed() && robot.exampleSubsystem != null) robot.exampleSubsystem.toggle().schedule();
        if (gamepad1.squareWasPressed()) robot.drivetrain.lockHeading(Alliance.current == Alliance.RED ? 90 : 270);
        // Vision-enabled profile only; correction is reserved for fixed, surveyed practice tags.
        if (gamepad1.circleWasPressed() && robot.limelight != null) robot.limelight.relocalize().schedule();

        // Operator overrides:
        if (gamepad2.leftBumperWasPressed()) Alliance.current = Alliance.RED;
        if (gamepad2.rightBumperWasPressed()) Alliance.current = Alliance.BLUE;
        if (gamepad2.crossWasPressed() && FieldConstants.START_POSE_CONFIGURED) {
            robot.drivetrain.setPose(Alliance.current == Alliance.RED
                    ? FieldConstants.RED_CORNER_START : mirror(FieldConstants.RED_CORNER_START));
        }
        if (!FieldConstants.START_POSE_CONFIGURED) {
            robot.telemetry.addLine("Corner reset disabled: measure the start pose and set START_POSE_CONFIGURED.");
        }

        robot.telemetry.addData("Alliance", Alliance.current);
    }
}
