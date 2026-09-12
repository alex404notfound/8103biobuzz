package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

/** Select teamconfig8103_vision; no Expansion Hub/example mechanism is required. */
@TeleOp(name = "Skeleton Vision TeleOp", group = "Competition")
public class SkeletonVisionTeleOp extends SkeletonTeleOp {
    @Override protected Robot createRobot() {
        return new Robot(this, Robot.HardwareProfile.DRIVE_AND_VISION);
    }
}
