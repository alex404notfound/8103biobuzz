package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.function.LongSupplier;

/** Runs one timed robot-relative move using the configured drive motors. */
abstract class TimedDriveAuto extends OpMode {
    private static final int MAX_MOVE_MS = 5000;

    private final LongSupplier clock;
    private DcMotorEx[] motors = new DcMotorEx[0];
    private Mecanum drive;
    private DrivePowers powers;
    private long moveNanos, startedAt;
    private boolean ready, moving;

    TimedDriveAuto(LongSupplier clock) { this.clock = clock; }

    final void initMove(int duration, double forward, double left, String readyMessage) {
        ready = moving = false;
        try {
            MecanumConfig config = Constants.drivetrainConfig;
            motors = new DcMotorEx[]{
                    hardwareMap.get(DcMotorEx.class, config.frontLeftName.get()),
                    hardwareMap.get(DcMotorEx.class, config.frontRightName.get()),
                    hardwareMap.get(DcMotorEx.class, config.backLeftName.get()),
                    hardwareMap.get(DcMotorEx.class, config.backRightName.get())
            };
            MotorSafety.stopAndBrake(motors);
            if (new HashSet<>(Arrays.asList(motors)).size() != motors.length)
                throw new IllegalArgumentException("Configure four distinct drive motors");
            for (DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive = Constants.createDrivetrain(hardwareMap);
            drive.stop(true);

            double totalPower = Math.abs(forward) + Math.abs(left);
            ready = duration > 0 && duration <= MAX_MOVE_MS && totalPower > 0 && totalPower <= 1;
            moveNanos = duration * 1_000_000L;
            powers = new DrivePowers(forward, left, 0);
            telemetry.addData("Move duration (ms)", duration);
            telemetry.addData("Forward / left power", "%.2f / %.2f", forward, left);
            telemetry.addLine(ready ? readyMessage
                    : "Set moveMs to 1–5000 and total absolute drive power to (0, 1], then re-INIT.");
            telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void start() {
        if (!ready) return;
        ready = false;
        try {
            startedAt = clock.getAsLong();
            drive.drive(powers, true);
            moving = true;
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void loop() {
        if (!moving) return;
        long elapsed = clock.getAsLong() - startedAt;
        if (elapsed >= moveNanos || elapsed < 0) {
            stop();
            telemetry.addLine("Drive stopped");
            telemetry.update();
        }
    }

    @Override public void stop() {
        ready = moving = false;
        MotorSafety.stopAndBrake(motors);
    }

    private void shutdown(RuntimeException failure) {
        try { stop(); }
        catch (RuntimeException stopFailure) { if (failure != stopFailure) failure.addSuppressed(stopFailure); }
        throw failure;
    }
}
