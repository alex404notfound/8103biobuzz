package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;


/** One-time sensor programming. See docs/BRUSHLAND_COLOR_TEST.md before rewiring. */
@Config
@TeleOp(name = "Brushland Configure", group = "Prototyping")
public class ConfigureColorRangefinder extends LinearOpMode {
    public enum OutputMode { DIGITAL_BIOBUZZ, ANALOG_HUE_CALIBRATION }

    public static OutputMode outputMode = OutputMode.DIGITAL_BIOBUZZ;
    // Manufacturer's previous red/yellow/blue example, NOT measured BIOBUZZ values.
    // Measure the actual balls with Brushland Hue Check, then adjust these degrees.
    public static double blueMinDegrees = 180, blueMaxDegrees = 250;
    public static double yellowMinDegrees = 55, yellowMaxDegrees = 90;
    public static double redMinDegrees = 0, redMaxDegrees = 50;
    public static int maxDistanceMm = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color"));
        telemetry.setMsTransmissionInterval(100);
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Output mode", outputMode);
            telemetry.addData("Blue / yellow / red hue (degrees)", "%.1f..%.1f / %.1f..%.1f / %.1f..%.1f",
                    blueMinDegrees, blueMaxDegrees, yellowMinDegrees, yellowMaxDegrees,
                    redMinDegrees, redMaxDegrees);
            telemetry.addData("Distance gate (mm)", maxDistanceMm);
            telemetry.addLine("I2C: REV Color Sensor V3 named Color. Reset before EACH programming run.");
            telemetry.addLine("START saves once. Defaults require calibration with actual BIOBUZZ balls.");
            telemetry.update();
            sleep(100);
        }
        waitForStart();
        if (isStopRequested()) return;
        try {
            writeConfiguration(crf);
        } catch (IllegalArgumentException invalid) {
            telemetry.addData("NOT WRITTEN", invalid.getMessage());
            telemetry.update();
            while (opModeIsActive()) sleep(100);
            return;
        }
        telemetry.addLine("Writes completed. Confirm the sensor's two LED blinks.");
        telemetry.addLine("STOP, then unplug/reconnect to the digital or analog port for the chosen mode.");
        telemetry.addLine("Use the matching robot config. Reset to I2C before programming again.");
        telemetry.update();
        while (opModeIsActive()) sleep(100);
    }

    static void writeConfiguration(ColorRangefinder crf) {
        // Snapshot and validate EVERYTHING before any persistent write.
        OutputMode mode = outputMode;
        int distance = maxDistanceMm;
        double[][] ranges = {{blueMinDegrees, blueMaxDegrees},
                {yellowMinDegrees, yellowMaxDegrees}, {redMinDegrees, redMaxDegrees}};
        if (mode == null) throw new IllegalArgumentException("Choose an output mode.");
        if (distance < 1 || distance > 100) throw new IllegalArgumentException("Distance must be 1..100 mm.");
        if (mode == OutputMode.ANALOG_HUE_CALIBRATION) {
            // Calibration must remain usable even while the digital ranges need fixing.
            crf.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
            // Both pins must leave I2C. Pin 1 isn't needed by the analog-only calibration reader.
            crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 255);
            crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, distance);
            return;
        }
        for (double[] range : ranges) {
            if (!Double.isFinite(range[0]) || !Double.isFinite(range[1])
                    || range[0] < 0 || range[0] >= 360 || range[1] <= 0 || range[1] > 360
                    || range[0] == range[1]) {
                throw new IllegalArgumentException("Hue endpoints must be distinct in 0..360 degrees.");
            }
            for (double[] interval : intervals(range)) {
                if (rawHue(interval[0]) == rawHue(interval[1])) {
                    throw new IllegalArgumentException("Hue range is narrower than sensor resolution.");
                }
            }
        }
        for (int i = 0; i < ranges.length; i++) {
            for (int j = i + 1; j < ranges.length; j++) {
                for (double[] a : intervals(ranges[i])) {
                    for (double[] b : intervals(ranges[j])) {
                        // Include 16-bit rounding and the circular 0/360 endpoint in the check.
                        if (Math.max(rawHue(a[0]), rawHue(b[0])) <= Math.min(rawHue(a[1]), rawHue(b[1]))
                                || (a[0] == 0 && b[1] == 360) || (b[0] == 0 && a[1] == 360)) {
                            throw new IllegalArgumentException("Color ranges must have gaps; overlapping ranges alias colors.");
                        }
                    }
                }
            }
        }
        writeHue(crf, true, ranges[0]);   // pin 0: blue OR yellow
        writeHue(crf, true, ranges[1]);
        crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, distance);
        writeHue(crf, false, ranges[2]);  // pin 1: red OR yellow
        writeHue(crf, false, ranges[1]);
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, distance);
    }

    private static long rawHue(double degrees) { return Math.round(degrees / 360.0 * 65535); }

    private static double[][] intervals(double[] range) {
        return range[0] < range[1] ? new double[][]{range}
                : new double[][]{{0, range[1]}, {range[0], 360}};
    }

    private static void writeHue(ColorRangefinder crf, boolean pin0, double[] range) {
        for (double[] interval : intervals(range)) {
            double lo = interval[0] / 360.0 * 255, hi = interval[1] / 360.0 * 255;
            if (pin0) crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, lo, hi);
            else crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, lo, hi);
        }
    }
}

/**
 * Helper class for configuring the Brushland Labs Color Rangefinder.
 * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
 */
class ColorRangefinder {
    // other writeable registers
    private static final byte CALIB_A_VAL_0 = 0x32;
    private static final byte PS_DISTANCE_0 = 0x42;
    private static final byte LED_BRIGHTNESS = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;
    private final I2cDeviceSynchSimple i2c;

    public ColorRangefinder(RevColorSensorV3 emulator) {
        this(emulator.getDeviceClient());
    }

    ColorRangefinder(I2cDeviceSynchSimple i2c) {
        this.i2c = i2c;
        this.i2c.enableWriteCoalescing(true);
    }

    public static int invertHue(int hue360) {
        return Math.floorMod(hue360 - 180, 360);
    }

    /**
     * Configure Pin 0 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
    }

    /**
     * Configure Pin 1 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 0's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin0DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin0Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 1's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin1DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin1Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin0InvertHue() {
        setPin0DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin1InvertHue() {
        setPin1DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * The denominator is what the raw sensor readings will be divided by before being scaled to 12-bit analog.
     * For the full range of that channel, leave the denominator as 65535 for colors or 100 for distance.
     * Smaller values will clip off higher ranges of the data in exchange for higher resolution within a lower range.
     */
    public void setPin0Analog(AnalogMode analogMode, int denominator) {
        byte denom0 = (byte) (denominator & 0xFF);
        byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
        i2c.write(PinNum.PIN0.modeAddress, new byte[]{analogMode.value, denom0, denom1});
    }

    /**
     * Configure Pin 0 as analog output of one of the six data channels.
     * To read analog, make sure the physical switch on the sensor is flipped away from the
     * connector side.
     */
    public void setPin0Analog(AnalogMode analogMode) {
        setPin0Analog(analogMode, analogMode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    public float[] getCalibration() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    /**
     * Save a brightness value of the LED to the sensor.
     *
     * @param value brightness between 0-255
     */
    public void setLedBrightness(int value) {
        i2c.write8(LED_BRIGHTNESS, value);
    }

    /**
     * Change the I2C address at which the sensor will be found. The address can be reset to the
     * default of 0x52 by holding the reset button.
     *
     * @param value new I2C address from 1 to 127
     */
    public void setI2cAddress(int value) {
        i2c.write8(I2C_ADDRESS_REG, value << 1);
    }

    /**
     * Read distance via I2C
     *
     * @return distance in millimeters
     */
    public double readDistance() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(PS_DISTANCE_0, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return bytes.getFloat();
    }

    private void setDigital(
            PinNum pinNum,
            DigitalMode digitalMode,
            double lowerBound,
            double higherBound
    ) {
        int lo, hi;
        if (lowerBound == higherBound) {
            lo = (int) lowerBound;
            hi = (int) higherBound;
        } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
            lo = (int) Math.round(lowerBound / 255.0 * 65535);
            hi = (int) Math.round(higherBound / 255.0 * 65535);
        } else { // distance in mm
            float[] calib = getCalibration();
            if (lowerBound < .5) hi = 2048;
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
        }

        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        i2c.write(pinNum.modeAddress, new byte[]{digitalMode.value, lo0, lo1, hi0, hi1});
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
        }
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);

        private final byte modeAddress;

        PinNum(int modeAddress) {
            this.modeAddress = (byte) modeAddress;
        }
    }

    public enum DigitalMode {
        RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);
        public final byte value;

        DigitalMode(int value) {
            this.value = (byte) value;
        }
    }

    public enum AnalogMode {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
        public final byte value;

        AnalogMode(int value) {
            this.value = (byte) value;
        }
    }
}
