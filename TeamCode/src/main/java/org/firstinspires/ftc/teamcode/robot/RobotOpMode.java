package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/** Owns loop ordering and cleanup; OpModes implement the protected hooks. */
public abstract class RobotOpMode extends OpMode {
    protected Robot robot;
    private long previousLoopNanos;

    @Override
    public final void init() {
        Scheduler.reset();
        previousLoopNanos = 0;
        robot = createRobot();
        onInit();
    }

    @Override
    public final void init_loop() {
        robot.clearBulkCache();
        onInitLoop();
        Scheduler.execute();
        robot.periodic();
        robot.updateTelemetry(Double.NaN);
    }

    @Override
    public final void start() {
        previousLoopNanos = System.nanoTime();
        onStart();
    }

    @Override
    public final void loop() {
        robot.clearBulkCache();
        onLoop();
        Scheduler.execute();
        robot.periodic();

        long now = System.nanoTime();
        double loopTimeMs = previousLoopNanos == 0 ? Double.NaN : (now - previousLoopNanos) / 1e6;
        previousLoopNanos = now;
        robot.updateTelemetry(loopTimeMs);
    }

    @Override
    public final void stop() {
        try {
            onStop();
        } finally {
            try {
                if (robot != null) robot.stop();
            } finally {
                // Ivy reset clears bookkeeping; Robot.stop explicitly stops the hardware.
                Scheduler.reset();
            }
        }
    }

    protected Robot createRobot() { return new Robot(this); }
    protected void onInit() { }
    protected void onInitLoop() { }
    protected void onStart() { }
    protected void onLoop() { }
    protected void onStop() { }
}
