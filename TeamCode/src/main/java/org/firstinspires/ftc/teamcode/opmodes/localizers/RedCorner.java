package org.firstinspires.ftc.teamcode.opmodes.localizers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous(name = "Red Corner", group = "Localizers")
public class RedCorner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain.localize(FieldConstants.RED_CORNER_START);
        Alliance.current = Alliance.RED;
    }
}
