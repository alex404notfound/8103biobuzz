package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;

/** Run the single shooter continuously during power or RPM tuning. */
@Configurable
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuning extends ShooterTuningOpMode {
    public static ShooterFlywheel.SpeedTuning speed = ShooterFlywheel.SHOOTER;
    public static volatile ControlMode controlMode = ControlMode.POWER;
    public static volatile double power = 0.25;
    public static volatile double intakePower, transferPower;
    public static volatile int shotStopperBlocked = 1;

    public FlywheelTuning() { super(Constants.SHOOTER_MOTOR_NAME, "shooter"); }

    @Override ShooterFlywheel.SpeedTuning speedTuning() { return speed; }

    @Override void resetAuxiliaryControls() {
        intakePower = transferPower = 0;
        shotStopperBlocked = 1;
    }

    @Override void updateTuning() {
        applyTuning(controlMode, power, intakePower, transferPower, shotStopperBlocked);
    }
}
