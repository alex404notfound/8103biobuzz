package org.firstinspires.ftc.teamcode.opmodes.testing;

import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

import static com.pedropathing.ivy.commands.Commands.instant;

/** Stationary diagnostic for camera/pipeline/frame setup, with explicit correction on Circle. */
@NextTeleop(name = "Limelight Check", group = "Diagnostics")
public class LimelightCheck extends RobotOpMode {
    public LimelightCheck(Robot robot) { super(robot); }

    @Override protected Robot.HardwareProfile hardwareProfile() {
        return Robot.HardwareProfile.DRIVE_AND_VISION;
    }

    @Override protected void onInit() { robot.drivetrain.usePreviousStartingPose(); }

    @Override protected void onStart() {
        driver.circle().onTrue(instant(() -> robot.limelight.relocalize().schedule()));
    }

    @Override protected void onInitLoop() {
        if (gamepad1.xWasPressed()) robot.drivetrain.recalibrateLocalization();
        robot.telemetry.addLine("Keep still. X recalibrates Pinpoint in INIT; wait for READY.");
        robot.telemetry.addLine("BIOBUZZ tags move. Absolute correction is for fixed, surveyed practice tags only.");
        robot.telemetry.addLine("For practice tags, verify camera map/extrinsics and Limelight frame before enabling correction.");
    }

    @Override protected void onLoop() {
        robot.telemetry.addLine("No driving controls. Circle requests a stationary translation correction.");
    }
}
