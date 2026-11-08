package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** One set of robot data shared by Driver Station and Panels. */
public final class RobotTelemetry {
    private RobotTelemetry() { }

    /** Standalone SDK OpModes flush this combined telemetry with their existing update(). */
    public static Telemetry forOpMode(Telemetry driver) {
        return new JoinedTelemetry(driver, browserTelemetry());
    }

    public static Telemetry browserTelemetry() {
        Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        panels.clearAll();
        return panels;
    }

    /** NextFTC owns the flush; register singleton backends instead of a per-INIT wrapper. */
    public static Telemetry forNextFTC(Telemetry driver) {
        Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        return forNextFTC(driver, panels);
    }

    static Telemetry forNextFTC(Telemetry driver, Telemetry panels) {
        panels.clearAll();
        dev.nextftc.robot.Telemetry.addBackend(panels);
        return new JoinedTelemetry(driver, panels);
    }
}
