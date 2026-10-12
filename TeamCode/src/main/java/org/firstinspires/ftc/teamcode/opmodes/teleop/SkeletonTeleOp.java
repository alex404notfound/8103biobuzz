package org.firstinspires.ftc.teamcode.opmodes.teleop;

import dev.nextftc.robot.opmode.NextTeleop;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;
import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * Drive-only starting point for the season's competition teleop.
 * Override onInit()/onLoop(); NextFTC owns the scheduler and mechanism updates.
 * Register CommandGamepad bindings in onStart(), after INIT calibration is done.
 * gamepad1 = driving + primary mechanism triggers; gamepad2 = operator overrides.
 */
@NextTeleop(name = "Skeleton TeleOp", group = "Competition")
public class SkeletonTeleOp extends RobotOpMode {

    public SkeletonTeleOp(Robot robot) { super(robot); }

    @Override
    protected void onInit() {
        robot.drivetrain.usePreviousStartingPose(); // carried over from auto / corner localizer
    }

    @Override
    protected void onStart() {
        if (robot.exampleSubsystem != null) driver.triangle().onTrue(robot.exampleSubsystem.toggle());
        driver.square().onTrue(instant(() -> robot.drivetrain.lockHeading(
                Alliance.current == Alliance.RED ? 90 : 270)).requiring(robot.drivetrain));
        // Build on each press so live Dashboard permission changes are respected.
        if (robot.limelight != null) driver.circle().onTrue(instant(() -> robot.limelight.relocalize().schedule()));
        operator.leftBumper().onTrue(instant(() -> Alliance.current = Alliance.RED));
        operator.rightBumper().onTrue(instant(() -> Alliance.current = Alliance.BLUE));
        operator.cross().onTrue(instant(() -> {
            if (FieldConstants.START_POSE_CONFIGURED) {
                robot.drivetrain.setPose(Alliance.current == Alliance.RED
                        ? FieldConstants.RED_CORNER_START : mirror(FieldConstants.RED_CORNER_START));
            }
        }).requiring(robot.drivetrain));
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

        if (!FieldConstants.START_POSE_CONFIGURED) {
            robot.telemetry.addLine("Corner reset disabled: measure the start pose and set START_POSE_CONFIGURED.");
        }

        robot.telemetry.addData("Alliance", Alliance.current);
    }
}
