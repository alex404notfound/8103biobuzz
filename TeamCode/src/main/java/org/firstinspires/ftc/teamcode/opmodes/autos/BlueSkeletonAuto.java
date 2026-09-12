package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "Blue Skeleton", group = "Competition")
public class BlueSkeletonAuto extends SkeletonAuto {
    public BlueSkeletonAuto() {
        super(Alliance.BLUE);
    }
}
