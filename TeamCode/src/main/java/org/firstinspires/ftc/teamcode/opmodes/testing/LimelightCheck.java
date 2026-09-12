package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

/** Stationary diagnostic for camera/pipeline/frame setup, with explicit correction on Circle. */
@TeleOp(name = "Limelight Check", group = "Diagnostics")
public class LimelightCheck extends RobotOpMode {
    @Override protected Robot createRobot() {
        return new Robot(this, Robot.HardwareProfile.DRIVE_AND_VISION);
    }

    @Override protected void onInit() { robot.drivetrain.usePreviousStartingPose(); }

    @Override protected void onInitLoop() {
        if (gamepad1.xWasPressed()) robot.drivetrain.recalibrateLocalization();
        robot.telemetry.addLine("Keep still. X recalibrates Pinpoint in INIT; wait for READY.");
        robot.telemetry.addLine("Verify camera map/extrinsics and Limelight frame settings before enabling correction.");
    }

    @Override protected void onLoop() {
        if (gamepad1.circleWasPressed()) robot.limelight.relocalize().schedule();
        robot.telemetry.addLine("No driving controls. Circle requests a stationary translation correction.");
    }
}
