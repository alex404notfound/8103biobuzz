package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.LongSupplier;

/** Linked two-motor flywheel and a positional rack-and-pinion compression servo. */
@Config
public class Launcher {
    // This reusable launcher uses ShooterFlywheel's voltage-based hardware and control settings.
    public static volatile String hoodServoName = "hood";
    public static final double TICKS_PER_REVOLUTION = ShooterFlywheel.TICKS_PER_REVOLUTION;
    public static final double MAX_MOTOR_RPM = ShooterFlywheel.MAX_MOTOR_RPM;

    // Initial bench speeds, not calibrated shooting presets. RPM refers to the motor shaft.
    public static volatile double smallBallRpm = 1000, largeBallRpm = 1000;

    // -1 means unmeasured. No servo position is written during construction, INIT, or STOP.
    public static volatile double hoodMin = -1, hoodMax = -1;
    public static volatile double smallBallHood = -1, largeBallHood = -1, calibrationHood = -1;
    public static volatile double hoodSettleMs = 300;

    public enum Ball { SMALL_POLLEN, LARGE_NECTAR }
    public enum Mode { OFF, VELOCITY, LEFT_TEST, RIGHT_TEST, FAULT }

    private final ShooterFlywheel flywheel;
    private final Servo hood;
    private final Telemetry telemetry;
    private final LongSupplier clock;
    private Ball ball = Ball.SMALL_POLLEN;
    private boolean enabled, closed;
    private double commandedHood = -1;
    private long hoodChangedAt;
    private String status = "STOPPED";

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this(new ShooterFlywheel(hardwareMap), hardwareMap.get(Servo.class, hoodServoName),
                telemetry, System::nanoTime);
    }

    Launcher(ShooterFlywheel flywheel, Servo hood, Telemetry telemetry, LongSupplier clock) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.telemetry = telemetry;
        this.clock = clock;
    }

    /** Called at START, never from INIT. Does not move anything. */
    public void enable() {
        if (closed) return;
        enabled = true;
        flywheel.enable();
    }

    public Ball getBall() { return ball; }

    public Mode getMode() {
        if (closed) return Mode.OFF;
        switch (flywheel.getMode()) {
            case VELOCITY: return Mode.VELOCITY;
            case LEFT_TEST: return Mode.LEFT_TEST;
            case RIGHT_TEST: return Mode.RIGHT_TEST;
            case FAULT: return Mode.FAULT;
            default: return Mode.OFF;
        }
    }

    public String getStatus() {
        return getMode() == Mode.OFF && status != null ? status : flywheel.getStatus();
    }
    public double getMeasuredRpm() { return flywheel.getMeasuredRpm(); }
    public double getCommandedHood() { return commandedHood; }
    public double getTargetRpm() { return ball == Ball.SMALL_POLLEN ? smallBallRpm : largeBallRpm; }
    private double presetHood() { return ball == Ball.SMALL_POLLEN ? smallBallHood : largeBallHood; }

    public void selectBall(Ball selected) {
        if (selected == null || selected == ball || closed) return;
        idle();
        ball = selected;
        status = "Ball changed; apply its hood preset with motors stopped";
    }

    /** Caller refreshes this request while its hold-to-run input is pressed. */
    public void requestVelocity() {
        if (!enabled || closed) return;
        status = null;
        flywheel.requestRpm(getTargetRpm());
    }

    /** Low-voltage direction check. The other motor coasts and may be back-driven by the linkage. */
    public void requestMotorTest(boolean testLeft) {
        if (!enabled || closed) return;
        status = null;
        flywheel.requestMotorTest(testLeft);
    }

    /** Release the run input to stop and acknowledge a latched flywheel fault. */
    public void idle() {
        if (closed) return;
        flywheel.idle();
        status = "STOPPED";
    }

    public boolean applyPresetHood() { return positionHood(presetHood()); }

    public boolean positionHood(double position) {
        if (!enabled || closed) { status = "Hood commands require START"; return false; }
        if (flywheel.getMode() != ShooterFlywheel.Mode.OFF || !flywheel.hasFreshSample()
                || !Double.isFinite(getMeasuredRpm()) || Math.abs(getMeasuredRpm()) > 150) {
            status = "Release motors and wait for the flywheel to coast down before moving hood";
            return false;
        }
        if (!validHoodPosition(position)) {
            status = "Measure hoodMin/hoodMax and the requested hood position first";
            return false;
        }
        hood.setPosition(position);
        if (Double.compare(position, commandedHood) != 0) hoodChangedAt = clock.getAsLong();
        commandedHood = position;
        status = "Hood commanded (settling is timed, no position sensor)";
        return true;
    }

    public boolean jogHood(double delta) {
        if (!Double.isFinite(delta)) return false;
        double origin = commandedHood >= 0 ? commandedHood : calibrationHood;
        if (!validHoodPosition(origin)) { status = "Set calibrationHood inside measured limits first"; return false; }
        return positionHood(Math.max(hoodMin, Math.min(hoodMax, origin + delta)));
    }

    public void adjustRpm(double delta) {
        if (closed || !Double.isFinite(delta) || !Double.isFinite(getTargetRpm())) return;
        double rpm = Math.max(0, Math.min(MAX_MOTOR_RPM, getTargetRpm() + delta));
        if (ball == Ball.SMALL_POLLEN) smallBallRpm = rpm;
        else largeBallRpm = rpm;
    }

    public boolean isAtSpeed() {
        return enabled && !closed && getMode() == Mode.VELOCITY && flywheel.isAtSpeed()
                && Double.compare(getTargetRpm(), flywheel.getTargetRpm()) == 0;
    }

    /** Mechanism status only: this is not permission to feed or proof that a hive cell is upward. */
    public boolean isReady() {
        return isAtSpeed() && validHoodPosition(presetHood())
                && Double.compare(commandedHood, presetHood()) == 0
                && finiteNonnegative(hoodSettleMs)
                && (clock.getAsLong() - hoodChangedAt) / 1e6 >= hoodSettleMs;
    }

    public void periodic() {
        if (closed) return;
        // Refresh a live Dashboard preset while running; the flywheel owns ramping and validation.
        if (enabled && getMode() == Mode.VELOCITY) flywheel.requestRpm(getTargetRpm());
        flywheel.periodic();
        report();
    }

    private static boolean validHoodPosition(double position) {
        return Double.isFinite(position) && Double.isFinite(hoodMin) && Double.isFinite(hoodMax)
                && hoodMin >= 0 && hoodMax <= 1 && hoodMin < hoodMax
                && position >= hoodMin && position <= hoodMax;
    }

    /** Terminal cleanup. Do not reposition the hood as part of STOP. */
    public void stop() {
        if (closed) return;
        enabled = false;
        closed = true;
        flywheel.stop();
    }

    private void report() {
        telemetry.addData("Launcher ball / mode", ball + " / " + getMode());
        telemetry.addData("Launcher target motor RPM", getTargetRpm());
        telemetry.addData("Launcher measured RPM", getMeasuredRpm());
        telemetry.addData("Launcher feedback encoder", flywheel.getFeedbackEncoder());
        telemetry.addData("Launcher at speed / preset ready", isAtSpeed() + " / " + isReady());
        telemetry.addData("Hood commanded / preset", "%.3f / %.3f", commandedHood, presetHood());
        telemetry.addData("Launcher status", getStatus());
    }

    public static double rpmToTicks(double rpm) { return ShooterFlywheel.rpmToTicks(rpm); }
    public static double ticksToRpm(double ticksPerSecond) { return ShooterFlywheel.ticksToRpm(ticksPerSecond); }
    private static boolean finiteNonnegative(double value) { return Double.isFinite(value) && value >= 0; }
}
