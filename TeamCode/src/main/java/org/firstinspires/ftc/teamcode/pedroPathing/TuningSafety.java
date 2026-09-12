package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/** Guards every selected Pedro tuner without changing the SDK selector's final lifecycle. */
final class TuningSafety extends OpMode {
    private static final long CALIBRATION_MIN_NANOS = 300_000_000L;
    private final Supplier<OpMode> factory;
    private final Follower follower;
    private final LongSupplier clock;
    private OpMode child;
    private boolean running;
    private boolean aborted;
    private boolean calibrationPending;
    private long calibrationStarted;
    private String status = "WAITING_FOR_SAMPLE";

    TuningSafety(Supplier<OpMode> factory, Follower follower) {
        this(factory, follower, System::nanoTime);
    }

    TuningSafety(Supplier<OpMode> factory, Follower follower, LongSupplier clock) {
        this.factory = factory;
        this.follower = follower;
        this.clock = clock;
    }

    @Override public void init() {
        // Child INIT can read the follower or zero offsets. Defer it until a healthy sample.
        stopDrive();
    }

    @Override public void init_loop() {
        if (aborted) { display(); return; }
        if (gamepad1.xWasPressed()) {
            stopChild();
            calibrationPending = true;
            calibrationStarted = clock.getAsLong();
            pinpoint().recalibrateIMU();
        }
        if (!sample()) {
            stopChild();
            display();
            return;
        }
        try {
            if (child == null) {
                child = factory.get();
                child.gamepad1 = gamepad1;
                child.gamepad2 = gamepad2;
                child.hardwareMap = hardwareMap;
                child.telemetry = telemetry;
                child.init();
            }
            child.init_loop();
        } catch (Constants.LocalizationNotReady fault) {
            status = fault.getMessage();
            // INIT can recover. Stop a partially initialized child to restore temporary offsets,
            // and construct a fresh one after localization becomes healthy again.
            stopChild();
        }
        display();
    }

    @Override public void start() {
        if (aborted) return;
        if (child == null || !sample()) {
            abort();
            return;
        }
        try {
            child.start();
            running = true;
        } catch (Constants.LocalizationNotReady fault) {
            status = fault.getMessage();
            abort();
        }
    }

    @Override public void loop() {
        if (aborted || !running) { display(); return; }
        // Catch B here: nested SDK OpModes do not receive the parent OpMode's stop services.
        if (gamepad1.b || gamepad1.bWasPressed()) {
            status = "Operator stopped tuning";
            abort();
            return;
        }
        if (!sample()) { abort(); return; }
        try {
            child.loop();
        } catch (Constants.LocalizationNotReady fault) {
            status = fault.getMessage();
            abort();
        }
    }

    /** A new sensor read precedes every child hook that may command or calculate motion. */
    private boolean sample() {
        try {
            follower.updatePose();
            follower.updateDrivetrain();
        } catch (Constants.LocalizationNotReady fault) {
            status = fault.getMessage();
            return false;
        }
        if (calibrationPending && clock.getAsLong() - calibrationStarted < CALIBRATION_MIN_NANOS) {
            status = "CALIBRATING - keep still";
            return false;
        }
        calibrationPending = false;
        status = "READY";
        return true;
    }

    private GoBildaPinpointDriver pinpoint() {
        return ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
    }

    private void abort() {
        aborted = true;
        running = false;
        stopChild();
        display();
    }

    private void stopChild() {
        OpMode previous = child;
        child = null;
        try {
            stopDrive();
        } finally {
            try {
                if (previous != null) previous.stop();
            } finally {
                stopDrive();
            }
        }
    }

    private void stopDrive() {
        follower.breakFollowing();
        // Follower.startTeleopDrive() performs a localization update; the hardware method does not.
        follower.drivetrain.startTeleopDrive(true);
    }

    private void display() {
        telemetry.addData("Tuning Pinpoint", status);
        telemetry.addLine(aborted ? "Tuning aborted. Press STOP; recovery will not restart motion."
                : "Keep still; X recalibrates Pinpoint in INIT. Wait for READY before START.");
        telemetry.update();
    }

    @Override public void stop() {
        aborted = true;
        running = false;
        stopChild();
    }
}
