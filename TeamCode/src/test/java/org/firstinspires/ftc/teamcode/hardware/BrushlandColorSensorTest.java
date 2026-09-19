package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.firstinspires.ftc.teamcode.hardware.BrushlandColorSensor.PieceColor.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class BrushlandColorSensorTest {
    private final DigitalChannel pin0 = mock(DigitalChannel.class);
    private final DigitalChannel pin1 = mock(DigitalChannel.class);
    private final AtomicLong now = new AtomicLong();
    private final BrushlandColorSensor sensor = new BrushlandColorSensor(pin0, pin1, 40, now::get);

    @Test public void configuresInputsAndDecodesActiveHighTruthTable() {
        verify(pin0).setMode(DigitalChannel.Mode.INPUT);
        verify(pin1).setMode(DigitalChannel.Mode.INPUT);
        assertFalse(sensor.hasSample());
        assertEquals(NONE_OR_UNKNOWN, sensor.getStableColor());
        assertEquals(NONE_OR_UNKNOWN, BrushlandColorSensor.decode(false, false));
        assertEquals(BLUE_NECTAR, BrushlandColorSensor.decode(true, false));
        assertEquals(RED_NECTAR, BrushlandColorSensor.decode(false, true));
        assertEquals(POLLEN, BrushlandColorSensor.decode(true, true));
    }

    @Test public void firstSampleMustRemainUnchangedForFortyMilliseconds() {
        sample(0, true, false);
        assertTrue(sensor.hasSample());
        assertEquals(BLUE_NECTAR, sensor.getRawColor());
        assertEquals(NONE_OR_UNKNOWN, sensor.getStableColor());
        for (int i = 0; i < 100; i++) sensor.update(); // Loop count cannot end the debounce.
        sample(39, true, false);
        assertEquals(NONE_OR_UNKNOWN, sensor.getStableColor());
        sample(40, true, false);
        assertFalse(sensor.isDebouncing());
        assertEquals(BLUE_NECTAR, sensor.getStableColor());
    }

    @Test public void transitionsInvalidateOldColorImmediatelyAndBounceRestartsTimer() {
        sample(0, false, true);
        sample(40, false, true);
        assertEquals(RED_NECTAR, sensor.getStableColor());
        sample(50, true, false);
        assertEquals(NONE_OR_UNKNOWN, sensor.getStableColor());
        assertFalse(sensor.isOwnNectar(Alliance.RED));
        sample(70, false, true); // A return to the previous color also needs a new dwell.
        sample(100, false, true);
        assertEquals(NONE_OR_UNKNOWN, sensor.getStableColor());
        sample(110, false, true);
        assertEquals(RED_NECTAR, sensor.getStableColor());
        sample(120, false, false);
        assertEquals(NONE_OR_UNKNOWN, sensor.getStableColor());
        sample(160, false, false);
        assertFalse(sensor.isDebouncing());
        assertEquals(NONE_OR_UNKNOWN, sensor.getStableColor());
    }

    @Test public void gettersNeverReadHardwareAndUpdateReadsEachPinOnlyOnce() {
        when(pin0.getState()).thenReturn(true);
        when(pin1.getState()).thenReturn(true);
        clearInvocations(pin0, pin1);
        sensor.update();
        for (int i = 0; i < 20; i++) {
            assertTrue(sensor.getPin0High());
            assertTrue(sensor.getPin1High());
            assertEquals(POLLEN, sensor.getRawColor());
            sensor.getStableColor(); sensor.hasSample(); sensor.isDebouncing();
            sensor.isOwnNectar(Alliance.RED); sensor.isOpponentNectar(Alliance.BLUE);
        }
        verify(pin0).getState();
        verify(pin1).getState();
        verifyNoMoreInteractions(pin0, pin1);
    }

    @Test public void allianceClassificationUsesOnlyStableNectarAndPollenIsNeutral() {
        sample(0, true, false); sample(40, true, false);
        assertTrue(sensor.isOwnNectar(Alliance.BLUE));
        assertTrue(sensor.isOpponentNectar(Alliance.RED));
        assertFalse(sensor.isOwnNectar(Alliance.RED));
        assertFalse(sensor.isOpponentNectar(Alliance.BLUE));
        sample(50, false, true); sample(90, false, true);
        assertTrue(sensor.isOwnNectar(Alliance.RED));
        assertTrue(sensor.isOpponentNectar(Alliance.BLUE));
        sample(100, true, true); sample(140, true, true);
        assertEquals(POLLEN, sensor.getStableColor());
        for (Alliance alliance : Alliance.values()) {
            assertFalse(sensor.isOwnNectar(alliance));
            assertFalse(sensor.isOpponentNectar(alliance));
        }
    }

    @Test public void debounceCanBeExplicitlyDisabledButCannotBeNegative() {
        BrushlandColorSensor immediate = new BrushlandColorSensor(pin0, pin1, 0, now::get);
        when(pin1.getState()).thenReturn(true);
        immediate.update();
        assertEquals(RED_NECTAR, immediate.getStableColor());
        assertThrows(IllegalArgumentException.class,
                () -> new BrushlandColorSensor(pin0, pin1, -1, now::get));
    }

    private void sample(long milliseconds, boolean high0, boolean high1) {
        now.set(milliseconds * 1_000_000L);
        when(pin0.getState()).thenReturn(high0);
        when(pin1.getState()).thenReturn(high1);
        sensor.update();
    }
}
