package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import kotlin.Unit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

/** Runs the published Panels SDK wrapper and NextFTC backend registry together. */
public class RobotTelemetryTest {
    private Set<Object> backends;
    private Set<Object> originalBackends;
    private final Telemetry driver = mock(Telemetry.class);
    private final List<List<String>> frames = new ArrayList<>();
    private TelemetryManager panels;

    @SuppressWarnings("unchecked")
    @Before public void setUp() throws Exception {
        Field field = dev.nextftc.robot.Telemetry.class.getDeclaredField("backends");
        field.setAccessible(true);
        backends = (Set<Object>) field.get(null);
        originalBackends = new LinkedHashSet<>(backends);
        TelemetryPluginConfig config = new TelemetryPluginConfig();
        config.setTelemetryUpdateInterval(0);
        panels = new TelemetryManager(() -> config,
                lines -> { frames.add(new ArrayList<>(lines)); return Unit.INSTANCE; },
                ms -> { config.setTelemetryUpdateInterval(ms); return Unit.INSTANCE; });
    }

    @After public void restoreRegistry() {
        backends.clear();
        backends.addAll(originalBackends);
    }

    @Test public void reinitializationDoesNotDuplicatePanelsFlushes() {
        RobotTelemetry.forNextFTC(driver, panels.getWrapper());
        Telemetry combined = RobotTelemetry.forNextFTC(driver, panels.getWrapper());
        assertEquals(originalBackends.size() + 1, backends.size());
        combined.addData("shooter.big.measuredRpm", 2400.5);
        combined.addLine("READY");
        assertTrue(frames.isEmpty()); // Adding data never steals NextFTC's end-of-loop flush.
        dev.nextftc.robot.Telemetry.update();
        assertEquals(1, frames.size());
        assertTrue(frames.get(0).contains("shooter.big.measuredRpm: 2400.5"));
        assertTrue(frames.get(0).contains("READY"));
        verify(driver).addData("shooter.big.measuredRpm", 2400.5);
    }

    @Test public void nextInitDiscardsBufferedDataFromThePreviousOpMode() {
        Telemetry previous = RobotTelemetry.forNextFTC(driver, panels.getWrapper());
        previous.addData("Old shooter target", 5000);
        Telemetry next = RobotTelemetry.forNextFTC(driver, panels.getWrapper());
        next.addData("New target", 0);
        dev.nextftc.robot.Telemetry.update();
        assertEquals(java.util.Collections.singletonList("New target: 0"), frames.get(0));
    }
}
