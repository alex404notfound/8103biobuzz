package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.*;

/** Checks the persistent protocol and prevents bad settings being partially written. */
public class ConfigureColorRangefinderTest {
    @Before public void setUp() { defaults(); }
    @After public void tearDown() { defaults(); }

    private static void defaults() {
        ConfigureColorRangefinder.outputMode = ConfigureColorRangefinder.OutputMode.DIGITAL_BIOBUZZ;
        ConfigureColorRangefinder.blueMinDegrees = 180;
        ConfigureColorRangefinder.blueMaxDegrees = 250;
        ConfigureColorRangefinder.yellowMinDegrees = 55;
        ConfigureColorRangefinder.yellowMaxDegrees = 90;
        ConfigureColorRangefinder.redMinDegrees = 0;
        ConfigureColorRangefinder.redMaxDegrees = 50;
        ConfigureColorRangefinder.maxDistanceMm = 20;
    }

    @Test public void defaultProfileWritesTwoColorUnionsAndDistanceGates() {
        I2cDeviceSynchSimple i2c = mock(I2cDeviceSynchSimple.class);
        ColorRangefinder crf = new ColorRangefinder(i2c);
        ConfigureColorRangefinder.writeConfiguration(crf);
        InOrder writes = inOrder(i2c);
        writes.verify(i2c).enableWriteCoalescing(true);
        // Hue 180 deg = 32768, 250 deg = 45510. All multibyte values little-endian.
        writes.verify(i2c).write(0x28, new byte[]{5, 0, (byte) 0x80, (byte) 0xc6, (byte) 0xb1});
        writes.verify(i2c).write(0x28, huePacket(55, 90));
        writes.verify(i2c).write(0x28, new byte[]{5, 20, 0, 20, 0});
        writes.verify(i2c).write(0x2d, huePacket(0, 50));
        writes.verify(i2c).write(0x2d, huePacket(55, 90));
        writes.verify(i2c).write(0x2d, new byte[]{5, 20, 0, 20, 0});
        verifyNoMoreInteractions(i2c);
    }

    @Test public void redCanCrossZeroWithoutInvertingOtherColors() {
        ColorRangefinder crf = mock(ColorRangefinder.class);
        ConfigureColorRangefinder.redMinDegrees = 340;
        ConfigureColorRangefinder.redMaxDegrees = 30;
        ConfigureColorRangefinder.writeConfiguration(crf);
        verify(crf).setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 30 / 360.0 * 255);
        verify(crf).setPin1Digital(ColorRangefinder.DigitalMode.HSV, 340 / 360.0 * 255, 255);
        verify(crf, never()).setPin1InvertHue();
    }

    @Test public void analogCalibrationAlsoConfiguresSecondPin() {
        ColorRangefinder crf = mock(ColorRangefinder.class);
        ConfigureColorRangefinder.outputMode = ConfigureColorRangefinder.OutputMode.ANALOG_HUE_CALIBRATION;
        ConfigureColorRangefinder.redMaxDegrees = 55; // Irrelevant overlapping digital settings.
        ConfigureColorRangefinder.writeConfiguration(crf);
        verify(crf).setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        verify(crf).setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 255);
        verify(crf).setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 20);
        verifyNoMoreInteractions(crf);
    }

    @Test public void invalidOrOverlappingThresholdsNeverPartiallyProgramSensor() {
        ColorRangefinder crf = mock(ColorRangefinder.class);
        ConfigureColorRangefinder.redMaxDegrees = 55; // Touching yellow is ambiguous.
        assertThrows(IllegalArgumentException.class, () -> ConfigureColorRangefinder.writeConfiguration(crf));
        defaults();
        ConfigureColorRangefinder.yellowMinDegrees = Double.NaN;
        assertThrows(IllegalArgumentException.class, () -> ConfigureColorRangefinder.writeConfiguration(crf));
        defaults();
        ConfigureColorRangefinder.maxDistanceMm = 0;
        assertThrows(IllegalArgumentException.class, () -> ConfigureColorRangefinder.writeConfiguration(crf));
        defaults();
        ConfigureColorRangefinder.redMaxDegrees = 0.000001; // Invalid LAST color must not write blue first.
        assertThrows(IllegalArgumentException.class, () -> ConfigureColorRangefinder.writeConfiguration(crf));
        defaults();
        ConfigureColorRangefinder.redMinDegrees = 55;
        ConfigureColorRangefinder.redMaxDegrees = 55; // Equal endpoints are reserved protocol commands.
        assertThrows(IllegalArgumentException.class, () -> ConfigureColorRangefinder.writeConfiguration(crf));
        verifyNoInteractions(crf);
    }

    @Test public void wrappedRangesCannotOverlapOtherColors() {
        ColorRangefinder crf = mock(ColorRangefinder.class);
        ConfigureColorRangefinder.redMinDegrees = 340;
        ConfigureColorRangefinder.redMaxDegrees = 60;
        assertThrows(IllegalArgumentException.class, () -> ConfigureColorRangefinder.writeConfiguration(crf));
        verifyNoInteractions(crf);
    }

    @Test public void invertedHueAlwaysStaysOnPositiveWheel() {
        assertEquals(180, ColorRangefinder.invertHue(0));
        assertEquals(120, ColorRangefinder.invertHue(300));
    }

    private static byte[] huePacket(double lo, double hi) {
        int lower = (int) Math.round(lo / 360.0 * 65535);
        int upper = (int) Math.round(hi / 360.0 * 65535);
        return new byte[]{5, (byte) lower, (byte) (lower >> 8), (byte) upper, (byte) (upper >> 8)};
    }
}
