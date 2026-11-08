package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;

/** Run the small shooter continuously during power or RPM tuning. */
@Configurable
@TeleOp(name = "Small Shooter Tuning", group = "Tuning")
public class SmallShooterTuning extends ShooterTuningOpMode {
    public static ShooterFlywheel.SpeedTuning speed = ShooterFlywheel.SMALL_SHOOTER;
    public static volatile ControlMode controlMode = ControlMode.POWER;
    public static volatile double power = 0.25;
    public static volatile double intakePower, transferPower;
    public static volatile int shotStopperBlocked = 1;

    public SmallShooterTuning() { super("smallShooter", "smallShooter"); }

    @Override ShooterFlywheel.SpeedTuning speedTuning() { return speed; }

    @Override void resetAuxiliaryControls() {
        intakePower = transferPower = 0;
        shotStopperBlocked = 1;
    }

    @Override void updateTuning() {
        applyTuning(controlMode, power, intakePower, transferPower, shotStopperBlocked);
    }
}
