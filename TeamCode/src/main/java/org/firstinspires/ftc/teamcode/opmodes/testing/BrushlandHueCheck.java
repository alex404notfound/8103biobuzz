package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.LinkedHashMap;
import java.util.Map;

/** Reads the sensor's internal hue after configuring pin 0 for analog HSV output. */
@TeleOp(name = "Brushland Hue Check", group = "Diagnostics")
public class BrushlandHueCheck extends OpMode {
    private static final long TELEMETRY_PERIOD_NS = 100_000_000L;

    private final Map<LynxModule, LynxModule.BulkCachingMode> priorCacheModes =
            new LinkedHashMap<>();
    private AnalogInput hueInput;
    private long lastTelemetryNs;
    private long samples;
    private double minimumHue = Double.POSITIVE_INFINITY;
    private double maximumHue = Double.NEGATIVE_INFINITY;
    private boolean previousB;

    @Override public void init() {
        hueInput = hardwareMap.get(AnalogInput.class, "colorHue");
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            priorCacheModes.put(hub, hub.getBulkCachingMode());
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("First configure ANALOG_HUE_CALIBRATION, then power-cycle into Analog Input.");
        telemetry.addLine("Sensor pin 0 must reach the Analog Input named colorHue.");
        telemetry.addLine("Hold A to capture one ball's hue; B clears captured values.");
        telemetry.update();
    }

    @Override public void init_loop() {
        sampleAndDisplay();
    }

    @Override public void loop() {
        sampleAndDisplay();
    }

    private void sampleAndDisplay() {
        for (LynxModule hub : priorCacheModes.keySet()) {
            hub.clearBulkCache();
        }
        double voltage = hueInput.getVoltage();
        // Brushland defines its full analog hue range as 0..3.3 V, not the hub ADC maximum.
        double hue = voltage / 3.3 * 360.0;
        boolean clearPressed = gamepad1.b && !previousB;
        previousB = gamepad1.b;
        if (clearPressed) {
            samples = 0;
            minimumHue = Double.POSITIVE_INFINITY;
            maximumHue = Double.NEGATIVE_INFINITY;
        }
        if (gamepad1.a && !gamepad1.b && Double.isFinite(hue)) {
            minimumHue = Math.min(minimumHue, hue);
            maximumHue = Math.max(maximumHue, hue);
            samples++;
        }

        long now = System.nanoTime();
        if (now - lastTelemetryNs < TELEMETRY_PERIOD_NS && !clearPressed) return;
        lastTelemetryNs = now;
        telemetry.addData("Voltage (V)", "%.4f", voltage);
        telemetry.addData("Internal hue (degrees)", "%.1f", hue);
        telemetry.addData("Capture", gamepad1.a && !gamepad1.b ? "RECORDING" : "Hold A");
        telemetry.addData("Captured samples", samples);
        if (samples > 0) {
            telemetry.addData("Captured min / max (degrees)", "%.1f / %.1f", minimumHue, maximumHue);
            if (maximumHue - minimumHue > 180.0) {
                telemetry.addLine("Possible 0/360 wrap: do not use this whole min-to-max interval.");
            }
        }
        telemetry.addLine("B: clear. Hold A while rotating ONE actual ball at its mounted sensing distance.");
        telemetry.addLine("Repeat for red nectar, blue nectar, and yellow pollen; record bounds plus margin.");
        telemetry.addLine("Red can cross 0/360: use separate intervals near 0 and 360.");
        telemetry.addLine("Hue alone does not establish object presence. These are calibration readings.");
        telemetry.update();
    }

    @Override public void stop() {
        for (Map.Entry<LynxModule, LynxModule.BulkCachingMode> entry : priorCacheModes.entrySet()) {
            entry.getKey().setBulkCachingMode(entry.getValue());
        }
        priorCacheModes.clear();
    }
}
