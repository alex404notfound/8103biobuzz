package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Pedro's single motor owner, with finite-output validation and an instance-local test power cap. */
public final class GuardedMecanum extends Mecanum {
    private double powerLimit = 1;

    public GuardedMecanum(HardwareMap map, MecanumConfig config) { super(map, config); }

    public double getPowerLimit() { return powerLimit; }

    public void setPowerLimit(double limit) {
        if (!Double.isFinite(limit) || limit <= 0 || limit > 1)
            throw new IllegalArgumentException("Motor power limit must be in (0, 1]");
        // A lower cap must not leave an earlier, larger motor command running.
        if (limit < powerLimit) stop();
        powerLimit = limit;
    }

    @Override public void drive(DrivePowers powers, boolean manual) {
        if (powers == null || !Double.isFinite(powers.forward())
                || !Double.isFinite(powers.strafe()) || !Double.isFinite(powers.turn())) {
            super.stop();
            throw new Constants.LocalizationNotReady("NON_FINITE_DRIVE_COMMAND");
        }
        if (powerLimit < 1) {
            // Each mecanum wheel is a signed sum of these axes. Preserve the requested ratio.
            double magnitude = Math.abs(powers.forward()) + Math.abs(powers.strafe()) + Math.abs(powers.turn());
            double scale = magnitude > powerLimit ? powerLimit / magnitude : 1;
            powers = new DrivePowers(powers.forward() * scale, powers.strafe() * scale, powers.turn() * scale);
        }
        super.drive(powers, manual);
    }
}
