package org.firstinspires.ftc.teamcode.opmodes.localizers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

// Measure the BIOBUZZ start pose, verify its blue equivalent, set START_POSE_CONFIGURED,
// then remove @Disabled.
@Disabled
@Autonomous(name = "Blue Corner", group = "Localizers")
public class BlueCorner extends LinearOpMode {
    @Override
    public void runOpMode() {
        if (!FieldConstants.START_POSE_CONFIGURED) {
            telemetry.addLine("Measure the BIOBUZZ start pose and set START_POSE_CONFIGURED first.");
            telemetry.update();
            return;
        }
        Drivetrain.localize(mirror(FieldConstants.RED_CORNER_START));
        Alliance.current = Alliance.BLUE;
    }
}
