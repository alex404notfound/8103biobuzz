package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import dev.nextftc.robot.opmode.NextAutonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Disabled // Pinpoint is currently enabled only for the standalone diagnostic.
@NextAutonomous(name = "Blue Skeleton", group = "Examples")
public class BlueSkeletonAuto extends SkeletonAuto {
    public BlueSkeletonAuto(Robot robot) {
        super(robot, Alliance.BLUE);
    }
}
