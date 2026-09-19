package org.firstinspires.ftc.teamcode.opmodes.localizers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

// Measure the BIOBUZZ start pose, set START_POSE_CONFIGURED, then remove @Disabled.
@Disabled
@Autonomous(name = "Red Corner", group = "Localizers")
public class RedCorner extends LinearOpMode {
    @Override
    public void runOpMode() {
        if (!FieldConstants.START_POSE_CONFIGURED) {
            telemetry.addLine("Measure the BIOBUZZ start pose and set START_POSE_CONFIGURED first.");
            telemetry.update();
            return;
        }
        Drivetrain.localize(FieldConstants.RED_CORNER_START);
        Alliance.current = Alliance.RED;
    }
}
