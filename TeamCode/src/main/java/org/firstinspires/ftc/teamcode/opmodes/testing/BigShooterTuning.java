package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFlywheel;

/** Run the big shooter continuously during power or RPM tuning. */
@Configurable
@TeleOp(name = "Big Shooter Tuning", group = "Tuning")
public class BigShooterTuning extends ShooterTuningOpMode {
    public static ShooterFlywheel.SpeedTuning speed =
            new ShooterFlywheel.SpeedTuning(0, 12.0 / ShooterFlywheel.MAX_MOTOR_RPM, false);
    public static volatile ControlMode controlMode = ControlMode.POWER;
    public static volatile double power = 0.25;
    public static volatile double intakePower, transferPower;
    public static volatile int shotStopperBlocked = 1;

    public BigShooterTuning() { super(Constants.BIG_SHOOTER_MOTOR_NAME, "bigShooter"); }

    @Override ShooterFlywheel.SpeedTuning speedTuning() { return speed; }

    @Override void resetAuxiliaryControls() {
        intakePower = transferPower = 0;
        shotStopperBlocked = 1;
    }

    @Override void updateTuning() {
        applyTuning(controlMode, power, intakePower, transferPower, shotStopperBlocked);
    }
}
