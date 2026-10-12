package org.firstinspires.ftc.teamcode.testutil;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.util.List;
import java.util.function.Supplier;

import static org.mockito.Mockito.mock;

/** Drives the real published NextFTC hooks, without an Android activity or a Control Hub. */
public final class NextOpModeHarness<T extends NextOpMode> implements AutoCloseable {
    public final T opMode;
    private final List<OpModeHook> hooks;

    @SuppressWarnings("unchecked")
    public NextOpModeHarness(Supplier<T> factory) {
        opMode = construct(factory);
        try {
            Field field = NextOpMode.class.getDeclaredField("hooks");
            field.setAccessible(true);
            hooks = (List<OpModeHook>) field.get(opMode);
        } catch (ReflectiveOperationException e) { throw new AssertionError(e); }
    }

    public static <T extends NextOpMode> T construct(Supplier<T> factory) {
        // These are the fields BoundNextOpMode sets before invoking the user constructor.
        active("activeGamepad1", new Gamepad());
        active("activeGamepad2", new Gamepad());
        active("activeHardwareMap", mock(HardwareMap.class));
        active("activeTelemetry", mock(Telemetry.class));
        try { return factory.get(); }
        finally {
            active("activeGamepad1", null);
            active("activeGamepad2", null);
            active("activeHardwareMap", null);
            active("activeTelemetry", null);
        }
    }

    private static void active(String name, Object value) {
        try {
            Field field = NextOpMode.class.getDeclaredField(name);
            field.setAccessible(true);
            field.set(null, value);
        } catch (ReflectiveOperationException e) { throw new AssertionError(e); }
    }

    public void init() { hooks.forEach(OpModeHook::afterConstruction); }
    public void initLoop() {
        hooks.forEach(OpModeHook::beforeDisabled);
        opMode.disabledPeriodic();
        hooks.forEach(OpModeHook::afterDisabled);
    }
    public void start() {
        hooks.forEach(OpModeHook::beforeStart);
        opMode.start();
        hooks.forEach(OpModeHook::afterStart);
    }
    public void loop() {
        hooks.forEach(OpModeHook::beforePeriodic);
        opMode.periodic();
        hooks.forEach(OpModeHook::afterPeriodic);
    }
    /** BoundNextOpMode invokes afterEnd even when it cannot reach the user's end(). */
    public void cleanupAfterFailure() {
        Throwable failure = null;
        for (OpModeHook hook : hooks) {
            try { hook.afterEnd(); }
            catch (Throwable error) {
                if (failure == null) failure = error;
                else failure.addSuppressed(error);
            }
        }
        if (failure != null) throw new AssertionError(failure);
    }
    @Override public void close() {
        try { opMode.end(); }
        finally { cleanupAfterFailure(); }
    }
}
