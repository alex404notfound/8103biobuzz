package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "Blue Skeleton", group = "Examples")
public class BlueSkeletonAuto extends SkeletonAuto {
    public BlueSkeletonAuto() {
        super(Alliance.BLUE);
    }
}
