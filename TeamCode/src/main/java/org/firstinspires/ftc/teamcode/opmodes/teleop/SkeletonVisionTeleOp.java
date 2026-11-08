package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.robot.Robot;

/** Select teamconfig8103_vision; no Expansion Hub/example mechanism is required. */
@Disabled
@NextTeleop(name = "Skeleton Vision TeleOp", group = "Competition")
public class SkeletonVisionTeleOp extends SkeletonTeleOp {
    public SkeletonVisionTeleOp(Robot robot) { super(robot); }

    @Override protected Robot.HardwareProfile hardwareProfile() {
        return Robot.HardwareProfile.DRIVE_AND_VISION;
    }
}
