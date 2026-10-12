package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/** NextFTC owns registration and scheduling; team hooks retain INIT sampling and safe cleanup. */
public abstract class RobotOpMode extends NextOpMode {
    protected final Robot robot;
    protected CommandGamepad driver;
    protected CommandGamepad operator;
    private long previousLoopNanos;
    private boolean started;
    private boolean ended;

    protected RobotOpMode(Robot robot) { this(robot, new LifecycleHook()); }

    private RobotOpMode(Robot robot, LifecycleHook lifecycle) {
        super(robot, lifecycle);
        this.robot = robot;
        lifecycle.owner = this;
    }

    @Override public final void disabledPeriodic() {
        if (!ended) onInitLoop();
    }

    @Override public final void start() {
        if (ended) return;
        started = true;
        previousLoopNanos = System.nanoTime();
        onStart();
    }

    @Override public final void periodic() {
        if (started && !ended) onLoop();
    }

    @Override public final void end() { finish(); }

    private void finish() {
        if (ended) return;
        ended = true;
        started = false;
        try {
            onStop();
        } finally {
            try {
                robot.stop();
            } finally {
                Trigger.Companion.getDefaultEventLoop().clear();
                Scheduler.reset();
            }
        }
    }

    /** BoundNextOpMode 0.2.1 calls start even when STOP releases waitForStart. */
    protected boolean isStartAllowed() {
        OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        OpMode active = manager == null ? null : manager.getActiveOpMode();
        return active instanceof LinearOpMode && !((LinearOpMode) active).isStopRequested();
    }

    protected Robot.HardwareProfile hardwareProfile() { return Robot.HardwareProfile.DRIVE_ONLY; }
    protected void onInit() { }
    protected void onInitLoop() { }
    protected void onStart() { }
    protected void onLoop() { }
    /** Runs after this loop's sensor/control updates and before NextFTC polls triggers/schedules commands. */
    protected void onAfterMechanisms() { }
    protected void onStop() { }

    private static final class LifecycleHook implements OpModeHook {
        private RobotOpMode owner;

        @Override public void afterConstruction() {
            Scheduler.reset();
            Trigger.Companion.getDefaultEventLoop().clear();
            owner.robot.initialize(owner.hardwareMap, owner.telemetry, owner.hardwareProfile());
            owner.driver = new CommandGamepad(owner.gamepad1);
            owner.operator = new CommandGamepad(owner.gamepad2);
            owner.robot.setAfterMechanisms(() -> {
                if (owner.started && !owner.ended) owner.onAfterMechanisms();
                long now = System.nanoTime();
                double elapsed = owner.started ? (now - owner.previousLoopNanos) / 1e6 : Double.NaN;
                owner.previousLoopNanos = now;
                owner.robot.updateTelemetry(elapsed);
            });
            owner.onInit();
        }

        @Override public void beforeDisabled() { owner.robot.clearBulkCache(); }
        @Override public void afterDisabled() { owner.robot.updateMechanismsInInit(); }
        @Override public void beforePeriodic() { owner.robot.clearBulkCache(); }
        @Override public void beforeStart() {
            if (!owner.isStartAllowed()) owner.finish();
        }
        // NextFTC guarantees afterEnd even when INIT, START, or a loop throws.
        @Override public void afterEnd() { owner.finish(); }
    }
}
