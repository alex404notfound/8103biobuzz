package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AxonTurret;

import java.util.Collections;
import java.util.List;

/** Standalone practice test: no drivetrain, launcher, Pinpoint or camera required. */
@Config
@TeleOp(name = "Axon Turret Test", group = "Prototyping")
public class AxonTurretTest extends OpMode {
    public static volatile double targetDegrees = 0, targetStepDegrees = 5;
    private AxonTurret turret;
    private List<LynxModule> hubs = Collections.emptyList();
    private boolean active, stopped, releaseRequired;

    public AxonTurretTest() { }

    AxonTurretTest(AxonTurret turret, Telemetry telemetry) {
        this.turret = turret;
        this.telemetry = telemetry;
    }

    @Override public void init() {
        active = false; stopped = false; releaseRequired = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
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
        telemetry.addLine("INIT: raw encoder readout only. Configure AxonTurret in Dashboard.");
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
            // so a B press during motion cannot unexpectedly zero later when controls release.
            boolean zeroPressed = gamepad1.bWasPressed(), clearPressed = gamepad1.yWasPressed();
            boolean originPressed = gamepad1.aWasPressed();
            boolean leftPressed = gamepad1.dpadLeftWasPressed(), rightPressed = gamepad1.dpadRightWasPressed();
            if (gamepad1.x) releaseRequired = true;
            if (releaseRequired) {
                turret.idle();
                if (motionReleased() && !gamepad1.x && !gamepad1.b && !gamepad1.y) releaseRequired = false;
            } else {
                if (originPressed) targetDegrees = 0;
                if (Double.isFinite(targetStepDegrees) && targetStepDegrees > 0 && targetStepDegrees <= 30) {
                    if (leftPressed) targetDegrees -= targetStepDegrees;
                    if (rightPressed) targetDegrees += targetStepDegrees;
                }
                if (gamepad1.left_bumper) {
                    double stick = Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x;
                    turret.requestManual(stick); // Manual overrides PID while LB is held.
                } else if (gamepad1.right_trigger > 0.25) turret.requestPosition(targetDegrees);
                else turret.idle();

                // Referencing or acknowledging a fault cannot also request motion.
                if (motionReleased()) {
                    if (clearPressed) turret.clearFault();
                    if (zeroPressed) { turret.zeroHere(); targetDegrees = 0; }
                }
            }
            tick();
            if (releaseRequired) telemetry.addLine("Release RT/LB, center stick, and release X/B/Y to arm controls.");
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
        for (LynxModule hub : hubs) hub.clearBulkCache();
        turret.periodic();
        telemetry.addData("Turret mode / status", turret.getMode() + " / " + turret.getStatus());
        telemetry.addData("Encoder volts / wrapped shaft deg", "%.3f / %.2f", turret.getVoltage(), turret.getWrappedDegrees());
        telemetry.addData("Unwrapped signed shaft deg", turret.getShaftDegrees());
        telemetry.addData("Zero established", turret.hasReference());
        telemetry.addData("Turret deg / deg per second", "%.2f / %.2f", turret.getPositionDegrees(), turret.getVelocityDegreesPerSecond());
        telemetry.addData("Target deg / error deg", "%.2f / %.2f", targetDegrees, turret.getErrorDegrees());
        // Separate numeric series can be graphed directly in Dashboard during tuning.
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
        telemetry.addData("Servo turns per turret turn", AxonTurret.servoTurnsPerTurretTurn);
        telemetry.addData("Travel limits", AxonTurret.limitTravel ? AxonTurret.minDegrees + " .. " + AxonTurret.maxDegrees : "DISABLED (continuous travel)");
        telemetry.addLine("LB + left stick X: manual jog | RT: hold profiled PID + feedforward | X: cancel");
        telemetry.addLine("B: mark physical zero | Y: clear fault (release all motion first)");
        telemetry.addLine("Left/right: target step | A: target zero | Dashboard: AxonTurret gains / AxonTurretTest target");
        telemetry.addLine("Practice only. RAW JOG has no travel limits until zero is established.");
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true; active = false;
        if (turret != null) turret.stop();
    }
}
