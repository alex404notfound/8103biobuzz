package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** One fixed-geometry shooter with separate RPM presets for the two ball sizes. */
@Configurable
public class Launcher {
    public static ShooterFlywheel.Settings shooter =
            new ShooterFlywheel.Settings(Constants.SHOOTER_MOTOR_NAME, ShooterFlywheel.SHOOTER);
    public static volatile double smallBallRpm = 1000, largeBallRpm = 1000;
    public enum Ball { SMALL_POLLEN, LARGE_NECTAR }

    private ShooterFlywheel flywheel;
    private final Telemetry telemetry;
    private Ball ball = Ball.SMALL_POLLEN;
    private boolean enabled, closed;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, shooter.motorName);
            motor.setPower(0);
            flywheel = new ShooterFlywheel(motor, shooter, hardwareMap);
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    Launcher(ShooterFlywheel flywheel, Telemetry telemetry) {
        this.flywheel = flywheel; this.telemetry = telemetry;
    }

    public void enable() {
        if (closed) return;
        enabled = true;
        flywheel.enable();
    }

    public void selectBall(Ball selected) {
        if (selected == null || selected == ball || closed) return;
        idle(); ball = selected;
    }

    public Ball getBall() { return ball; }
    public ShooterFlywheel selected() { return flywheel; }
    public double getManualRpm() { return ball == Ball.LARGE_NECTAR ? largeBallRpm : smallBallRpm; }
    public double getTargetRpm() { return selected().getTargetRpm(); }
    public double getMeasuredRpm() { return selected().getMeasuredRpm(); }
    public boolean isAtSpeed() { return enabled && !closed && selected().isAtSpeed(); }

    /** Caller refreshes this while its hold-to-run input is pressed, in manual or lookup mode. */
    public void requestRpm(double rpm) {
        if (enabled && !closed) selected().requestRpm(rpm);
    }

    public void requestMotorTest() {
        if (enabled && !closed) selected().requestMotorTest();
    }

    public void adjustRpm(double delta) {
        if (closed || !Double.isFinite(delta) || !Double.isFinite(getManualRpm())) return;
        double rpm = Math.max(0, Math.min(ShooterFlywheel.MAX_MOTOR_RPM, getManualRpm() + delta));
        if (ball == Ball.LARGE_NECTAR) largeBallRpm = rpm;
        else smallBallRpm = rpm;
    }

    public void idle() {
        if (closed) return;
        flywheel.idle();
    }

    public void periodic() {
        if (closed) return;
        try {
            flywheel.periodic();
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
        report(flywheel);
    }

    public void stop() {
        if (closed) return;
        closed = true; enabled = false;
        if (flywheel != null) flywheel.stop();
    }

    private void report(ShooterFlywheel shooter) {
        String key = "shooter.";
        telemetry.addData(key + "status", shooter.getStatus());
        telemetry.addData(key + "targetRpm", shooter.getTargetRpm());
        telemetry.addData(key + "measuredRpm", shooter.getMeasuredRpm());
        telemetry.addData(key + "referenceRpm", shooter.getReferenceRpm());
        telemetry.addData(key + "atSpeed", shooter.isAtSpeed());
        telemetry.addData(key + "power", shooter.getPower());
        telemetry.addData(key + "feedforwardVolts", shooter.getFeedforwardVolts());
        telemetry.addData(key + "feedbackVolts", shooter.getFeedbackVolts());
        telemetry.addData(key + "outputLimited", shooter.isOutputLimited());
    }
}
