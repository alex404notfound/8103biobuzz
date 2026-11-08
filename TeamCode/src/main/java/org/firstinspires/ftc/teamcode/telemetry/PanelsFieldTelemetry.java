package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.field.Canvas;
import com.bylazar.field.CanvasRotation;
import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.PanelsField;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;

import java.util.ArrayList;

/** Native Panels frames, in the team's red-left Pedro field coordinates. */
public final class PanelsFieldTelemetry {
    private PanelsFieldTelemetry() { }

    static Canvas newCanvas() {
        Canvas canvas = new Canvas();
        canvas.setPreset(new FieldPresetParams("BIOBUZZ / Pedro",
                -FieldConstants.FIELD_CENTER_X_INCHES, -FieldConstants.FIELD_CENTER_Y_INCHES,
                CanvasRotation.DEG_90, false, true, false));
        return canvas;
    }

    public static void send(Canvas canvas) {
        send(canvas, PanelsField.INSTANCE.getField());
    }

    static void send(Canvas canvas, FieldManager field) {
        // The native manager clears its working list after sending. Keep the input snapshot intact.
        field.setCanvas(new Canvas(canvas.getPreset(), new ArrayList<>(canvas.getItems()), field.getDefaultBgID()));
        // FieldMapTelemetry owns the 10 Hz limit and sends faults/recovery immediately.
        field.setLastUpdate(0);
        field.update();
    }
}
