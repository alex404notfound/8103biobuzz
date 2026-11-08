package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Two independent fixed-geometry shooters. Only the selected ball's motor is powered. */
@Configurable
public class Launcher {
    // Preserve the existing configuration names; verify this big/small assignment on the robot.
    public static ShooterFlywheel.Settings big = new ShooterFlywheel.Settings("launcherLeft");
    public static ShooterFlywheel.Settings small = new ShooterFlywheel.Settings("launcherRight", ShooterFlywheel.SMALL_SHOOTER);
    public static volatile double smallBallRpm = 1000, largeBallRpm = 1000;
    public enum Ball { SMALL_POLLEN, LARGE_NECTAR }

    private ShooterFlywheel bigShooter, smallShooter;
    private final Telemetry telemetry;
    private Ball ball = Ball.SMALL_POLLEN;
    private boolean enabled, closed;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            DcMotorEx bigMotor = hardwareMap.get(DcMotorEx.class, big.motorName);
            bigMotor.setPower(0);
            DcMotorEx smallMotor = hardwareMap.get(DcMotorEx.class, small.motorName);
            smallMotor.setPower(0);
            if (bigMotor == smallMotor) throw new IllegalArgumentException("Big and small shooters need distinct motors");
            bigShooter = new ShooterFlywheel(bigMotor, big, hardwareMap);
            smallShooter = new ShooterFlywheel(smallMotor, small, hardwareMap);
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    Launcher(ShooterFlywheel bigShooter, ShooterFlywheel smallShooter, Telemetry telemetry) {
        this.bigShooter = bigShooter; this.smallShooter = smallShooter; this.telemetry = telemetry;
    }

    public void enable() {
        if (closed) return;
        enabled = true;
        bigShooter.enable(); smallShooter.enable();
    }

    public void selectBall(Ball selected) {
        if (selected == null || selected == ball || closed) return;
        idle(); ball = selected;
    }

    public Ball getBall() { return ball; }
    public ShooterFlywheel selected() { return ball == Ball.LARGE_NECTAR ? bigShooter : smallShooter; }
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
        try { bigShooter.idle(); } finally { smallShooter.idle(); }
    }

    public void periodic() {
        if (closed) return;
        try {
            bigShooter.periodic(); smallShooter.periodic();
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
        report("big", bigShooter); report("small", smallShooter);
    }

    public void stop() {
        if (closed) return;
        closed = true; enabled = false;
        try { if (bigShooter != null) bigShooter.stop(); }
        finally { if (smallShooter != null) smallShooter.stop(); }
    }

    private void report(String name, ShooterFlywheel shooter) {
        String key = "shooter." + name + ".";
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
