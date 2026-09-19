package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.robot.Alliance;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

/** Sensor-only diagnostic. Requires two digital inputs and a REV hub; commands no actuators. */
@TeleOp(name = "Brushland Color Test", group = "Diagnostics")
public class BrushlandColorTest extends OpMode {
    private static final long TELEMETRY_PERIOD_NS = 100_000_000L;
    private BrushlandColorSensor sensor;
    private List<LynxModule> hubs;
    private final List<LynxModule.BulkCachingMode> previousModes = new ArrayList<>();
    private final LongSupplier clock;
    private Alliance alliance = Alliance.RED;
    private boolean started;
    private boolean stopped;
    private boolean hasCycle;
    private boolean displayed;
    private long previousCycleStart;
    private long lastDisplay;
    private long sampleCount;
    private long periodCount;
    private double totalReadNs;
    private double totalPeriodNs;
    private long maximumReadNs;
    private long maximumPeriodNs;

    public BrushlandColorTest() { clock = System::nanoTime; }

    // Keeps cache ordering and timing testable without a complete robot configuration.
    BrushlandColorTest(BrushlandColorSensor sensor, List<LynxModule> hubs,
                       Telemetry telemetry, LongSupplier clock) {
        this.sensor = sensor;
        this.hubs = hubs;
        this.telemetry = telemetry;
        this.clock = clock;
    }

    @Override public void init() {
        if (sensor == null) sensor = new BrushlandColorSensor(hardwareMap);
        if (hubs == null) hubs = hardwareMap.getAll(LynxModule.class);
        try {
            for (LynxModule hub : hubs) {
                previousModes.add(hub.getBulkCachingMode());
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } catch (RuntimeException failure) {
            restoreBulkModes();
            throw failure;
        }
        telemetry.setMsTransmissionInterval(100);
    }

    @Override public void init_loop() {
        if (stopped) return;
        if (!started) {
            if (gamepad1.dpad_left) alliance = Alliance.RED;
            else if (gamepad1.dpad_right) alliance = Alliance.BLUE;
        }
        sampleAndDisplay();
    }

    @Override public void start() {
        if (stopped) return;
        started = true;
        // Keep the already debounced sample; restart timing statistics for the running test.
        hasCycle = displayed = false;
        sampleCount = periodCount = maximumReadNs = maximumPeriodNs = 0;
        totalReadNs = totalPeriodNs = 0;
    }

    @Override public void loop() {
        if (!stopped) sampleAndDisplay();
    }

    private void sampleAndDisplay() {
        long cycleStart = clock.getAsLong();
        if (hasCycle) {
            long period = cycleStart - previousCycleStart;
            totalPeriodNs += period;
            maximumPeriodNs = Math.max(maximumPeriodNs, period);
            periodCount++;
        }
        hasCycle = true;
        previousCycleStart = cycleStart;

        // This is the ONLY cache clear for this cycle. Both digital channels share the cache.
        for (LynxModule hub : hubs) hub.clearBulkCache();
        sensor.update();
        long now = clock.getAsLong();
        long readDuration = now - cycleStart;
        totalReadNs += readDuration;
        maximumReadNs = Math.max(maximumReadNs, readDuration);
        sampleCount++;

        if (displayed && now - lastDisplay < TELEMETRY_PERIOD_NS) return;
        displayed = true;
        lastDisplay = now;
        telemetry.addData("Alliance", alliance);
        telemetry.addLine(started ? "Alliance locked for this run."
                : "INIT: D-pad LEFT = RED, RIGHT = BLUE. START locks alliance.");
        telemetry.addData("Sensor pin 0 / pin 1", "%d / %d",
                sensor.getPin0High() ? 1 : 0, sensor.getPin1High() ? 1 : 0);
        telemetry.addData("Raw color", sensor.getRawColor());
        telemetry.addData("Stable color", sensor.getStableColor());
        telemetry.addData("Debouncing (40 ms)", sensor.isDebouncing());
        telemetry.addData("Own / opponent nectar", "%s / %s",
                sensor.isOwnNectar(alliance), sensor.isOpponentNectar(alliance));
        telemetry.addData("Bulk clear + digital read avg / max (ms)", "%.3f / %.3f",
                totalReadNs / sampleCount / 1_000_000.0, maximumReadNs / 1_000_000.0);
        telemetry.addData("Loop period avg / max (ms)", "%.3f / %.3f",
                periodCount == 0 ? 0 : totalPeriodNs / periodCount / 1_000_000.0,
                maximumPeriodNs / 1_000_000.0);
        telemetry.addData("Samples", sampleCount);
        telemetry.addLine("00 = none/unknown; 10 = blue; 01 = red; 11 = yellow pollen.");
        telemetry.update();
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        restoreBulkModes();
    }

    private void restoreBulkModes() {
        // Save each mode before changing it so a partial initialization is also restorable.
        for (int i = 0; i < previousModes.size(); i++) {
            hubs.get(i).setBulkCachingMode(previousModes.get(i));
        }
        previousModes.clear();
    }
}
