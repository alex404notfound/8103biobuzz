package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.AxonTurret.GearPreset;

/** Maps counterclockwise turret degrees from calibrated zero to a positional servo command. */
@Configurable
public class ServoTurret {
    public static Settings big = new Settings("turret", GearPreset.BIG);
    public static Settings small = new Settings("smallTurret", GearPreset.SMALL);

    public static class Settings {
        public volatile String servoName;
        public volatile double zeroPosition = 0.5; // Servo command at the turret's physical zero.
        public volatile double servoSweepDegrees = 0; // Shaft degrees per unit of servo position.
        public volatile double minPosition = 0.5, maxPosition = 0.5; // Measure safe endpoints.
        public volatile boolean reversed = false;
        private final GearPreset gearing;

        Settings(String servoName, GearPreset gearing) {
            this.servoName = servoName;
            this.gearing = gearing;
        }
    }

    private final Servo servo;
    private final double zeroPosition, minPosition, maxPosition, degreesPerPosition;
    private final double minDegrees, maxDegrees;
    private double commandedDegrees = Double.NaN, commandedPosition = Double.NaN;

    /** Capture the selected turret's calibration at INIT. */
    public ServoTurret(HardwareMap map, Settings settings) {
        servo = map.get(Servo.class, settings.servoName);
        zeroPosition = settings.zeroPosition;
        minPosition = settings.minPosition;
        maxPosition = settings.maxPosition;
        double sweep = settings.servoSweepDegrees;
        degreesPerPosition = (settings.reversed ? -sweep : sweep) / settings.gearing.servoTurnsPerTurretTurn();
        boolean calibrated = sweep > 0 && Double.isFinite(degreesPerPosition)
                && minPosition >= 0 && minPosition <= zeroPosition && zeroPosition <= maxPosition
                && maxPosition <= 1 && minPosition < maxPosition;
        double first = (minPosition - zeroPosition) * degreesPerPosition;
        double last = (maxPosition - zeroPosition) * degreesPerPosition;
        minDegrees = calibrated ? Math.min(first, last) : Double.NaN;
        maxDegrees = calibrated ? Math.max(first, last) : Double.NaN;
    }

    /** An unreachable or nonfinite target leaves the last servo command in place. */
    public void setAngleDegrees(double degrees) {
        if (!isCalibrated() || !(degrees >= minDegrees && degrees <= maxDegrees)) return;
        double position = zeroPosition + degrees / degreesPerPosition;
        position = Math.max(minPosition, Math.min(maxPosition, position)); // Endpoint roundoff.
        servo.setPosition(position);
        commandedDegrees = degrees;
        commandedPosition = position;
    }

    public void nudgeDegrees(double delta) {
        if (!Double.isFinite(delta)) return;
        double start = Double.isFinite(commandedDegrees) ? commandedDegrees : 0;
        setAngleDegrees(Math.max(minDegrees, Math.min(maxDegrees, start + delta)));
    }

    public boolean isCalibrated() { return minDegrees < maxDegrees; }
    public double getMinDegrees() { return minDegrees; }
    public double getMaxDegrees() { return maxDegrees; }
    public double getCommandedDegrees() { return commandedDegrees; }
    public double getCommandedPosition() { return commandedPosition; }
}
