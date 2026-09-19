package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "Red Skeleton", group = "Examples")
public class RedSkeletonAuto extends SkeletonAuto {
    public RedSkeletonAuto() {
        super(Alliance.RED);
    }
}
