package org.firstinspires.ftc.teamcode.opmodes.autos;

import dev.nextftc.robot.opmode.NextAutonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

@NextAutonomous(name = "Blue Skeleton", group = "Examples")
public class BlueSkeletonAuto extends SkeletonAuto {
    public BlueSkeletonAuto(Robot robot) {
        super(robot, Alliance.BLUE);
    }
}
