package org.firstinspires.ftc.teamcode.opmodes.localizers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

@Autonomous(name = "Blue Corner", group = "Localizers")
public class BlueCorner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain.localize(mirror(FieldConstants.RED_CORNER_START));
        Alliance.current = Alliance.BLUE;
    }
}
