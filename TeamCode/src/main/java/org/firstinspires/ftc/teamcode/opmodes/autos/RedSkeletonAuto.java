package org.firstinspires.ftc.teamcode.opmodes.autos;

import dev.nextftc.robot.opmode.NextAutonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

@NextAutonomous(name = "Red Skeleton", group = "Examples")
public class RedSkeletonAuto extends SkeletonAuto {
    public RedSkeletonAuto(Robot robot) {
        super(robot, Alliance.RED);
    }
}
