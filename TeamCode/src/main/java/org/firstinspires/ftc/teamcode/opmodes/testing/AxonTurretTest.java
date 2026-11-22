package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.BulkCacheSession;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.AxonTurret;

/** Standalone practice test: no drivetrain, launcher, Pinpoint or camera required. */
@Configurable
@Disabled
@TeleOp(name = "Axon Turret Test", group = "Prototyping")
public class AxonTurretTest extends OpMode {
    public static volatile double targetDegrees = 0, targetStepDegrees = 5;
    private AxonTurret turret;
    private final BulkCacheSession bulkCache = new BulkCacheSession();
    private boolean active, stopped, releaseRequired;

    public AxonTurretTest() { }

    AxonTurretTest(AxonTurret turret, Telemetry telemetry) {
        this.turret = turret;
        this.telemetry = telemetry;
    }

    @Override public void init() {
        active = false; stopped = false; releaseRequired = true;
        telemetry = RobotTelemetry.forOpMode(telemetry);
        try {
            bulkCache.enable(hardwareMap);
            turret = new AxonTurret(hardwareMap);
            targetDegrees = 0;
        } catch (RuntimeException failure) {
            stop();
            throw failure;
        }
    }

    @Override public void init_loop() {
        if (stopped) return;
        tick();
        telemetry.addLine("INIT: raw encoder readout only. Configure AxonTurret in Panels.");
        telemetry.update();
    }

    @Override public void start() {
        if (stopped) return;
        active = true;
        releaseRequired = true;
        gamepad1.resetEdgeDetection();
        turret.enable();
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            // SDK button edges remain queued until consumed. Discard disallowed presses now,
            // so a Circle press during motion cannot unexpectedly zero later when controls release.
            boolean zeroPressed = gamepad1.circleWasPressed(), clearPressed = gamepad1.triangleWasPressed();
            boolean originPressed = gamepad1.crossWasPressed();
            boolean leftPressed = gamepad1.dpadLeftWasPressed(), rightPressed = gamepad1.dpadRightWasPressed();
            if (gamepad1.square) releaseRequired = true;
            if (releaseRequired) {
                turret.idle();
                if (motionReleased() && !gamepad1.square && !gamepad1.circle && !gamepad1.triangle) releaseRequired = false;
            } else {
                if (originPressed) targetDegrees = 0;
                if (Double.isFinite(targetStepDegrees) && targetStepDegrees > 0 && targetStepDegrees <= 30) {
                    if (leftPressed) targetDegrees -= targetStepDegrees;
                    if (rightPressed) targetDegrees += targetStepDegrees;
                }
                if (gamepad1.left_bumper) {
                    double stick = Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x;
                    turret.requestManual(stick); // Manual overrides PID while L1 is held.
                } else if (gamepad1.right_trigger > 0.25) turret.requestPosition(targetDegrees);
                else turret.idle();

                // Referencing or acknowledging a fault cannot also request motion.
                if (motionReleased()) {
                    if (clearPressed) turret.clearFault();
                    if (zeroPressed) { turret.zeroHere(); targetDegrees = 0; }
                }
            }
            tick();
            if (releaseRequired) telemetry.addLine("Release R2, L1, Square, Circle, and Triangle and center the stick to arm.");
            telemetry.update();
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }

    private boolean motionReleased() {
        return !gamepad1.left_bumper && gamepad1.right_trigger <= 0.25 && Math.abs(gamepad1.left_stick_x) < 0.1;
    }

    private void tick() {
        bulkCache.clear();
        turret.periodic();
        telemetry.addData("Turret mode / status", turret.getMode() + " / " + turret.getStatus());
        telemetry.addData("Encoder volts / wrapped shaft deg", "%.3f / %.2f", turret.getVoltage(), turret.getWrappedDegrees());
        telemetry.addData("Unwrapped signed shaft deg", turret.getShaftDegrees());
        telemetry.addData("Zero established", turret.hasReference());
        telemetry.addData("Turret deg / deg per second", "%.2f / %.2f", turret.getPositionDegrees(), turret.getVelocityDegreesPerSecond());
        telemetry.addData("Target deg / error deg", "%.2f / %.2f", targetDegrees, turret.getErrorDegrees());
        // Separate numeric series can be graphed directly in Panels during tuning.
        telemetry.addData("turret.actualDeg", turret.getPositionDegrees());
        telemetry.addData("turret.goalDeg", targetDegrees);
        telemetry.addData("turret.profileDeg", turret.getProfilePositionDegrees());
        telemetry.addData("turret.actualDegPerSec", turret.getVelocityDegreesPerSecond());
        telemetry.addData("turret.profileDegPerSec", turret.getProfileVelocityDegreesPerSecond());
        telemetry.addData("turret.profileDegPerSec2", turret.getProfileAccelerationDegreesPerSecondSquared());
        telemetry.addData("turret.feedbackPower", turret.getFeedbackPower());
        telemetry.addData("turret.feedforwardPower", turret.getFeedforwardPower());
        telemetry.addData("turret.requestedPower", turret.getUnclampedPower());
        telemetry.addData("turret.appliedPower", turret.getOutput() * AxonTurret.servoSign);
        telemetry.addData("Profile enabled / finished / duration s", AxonTurret.profileEnabled + " / "
                + turret.isProfileFinished() + " / " + turret.getProfileDurationSeconds());
        telemetry.addData("Output limited", turret.isOutputLimited());
        telemetry.addData("Power / at target", turret.getOutput() + " / " + turret.isAtTarget());
        telemetry.addData("Turret gear preset", turret.getGearPreset());
        telemetry.addData("Servo turns per turret turn", turret.getServoTurnsPerTurretTurn());
        telemetry.addData("Travel limits", AxonTurret.limitTravel ? AxonTurret.minDegrees + " .. " + AxonTurret.maxDegrees : "DISABLED (continuous travel)");
        telemetry.addLine("Hold L1 and move the left stick sideways to jog. Hold R2 to run the profile. Square cancels.");
        telemetry.addLine("With motion controls released, Circle marks physical zero and Triangle clears the fault.");
        telemetry.addLine("D-pad left/right steps the target. Cross sets it to zero. Tune gains and target in Panels.");
        telemetry.addLine("Practice only. RAW JOG has no travel limits until zero is established.");
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true; active = false;
        try { if (turret != null) turret.stop(); }
        finally { bulkCache.close(); }
    }
}
