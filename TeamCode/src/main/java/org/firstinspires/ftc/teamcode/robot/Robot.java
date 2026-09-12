package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.List;

public class Robot {
    public enum HardwareProfile { DRIVE_ONLY, DRIVE_AND_VISION, ALL }

    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final Drivetrain drivetrain;
    public final ExampleSubsystem exampleSubsystem;
    public final Limelight limelight;
    public final HardwareProfile profile;
    private final List<LynxModule> hubs;
    private boolean stopped;

    public Robot(OpMode opMode) {
        this(opMode, HardwareProfile.DRIVE_ONLY);
    }

    public Robot(OpMode opMode, HardwareProfile profile) {
        this(opMode.hardwareMap, new MultipleTelemetry(
                opMode.telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        ), profile);
    }

    private Robot(HardwareMap hardwareMap, Telemetry telemetry, HardwareProfile profile) {
        this(hardwareMap, telemetry, profile, new Drivetrain(hardwareMap, telemetry));
    }

    Robot(HardwareMap hardwareMap, Telemetry telemetry, HardwareProfile profile, Drivetrain drivetrain) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.profile = profile;

        // MANUAL bulk caching: each hub's sensor data (encoder positions/velocities,
        // digital inputs) is fetched in ONE bulk transfer per loop instead of one
        // ~3ms transfer per read. RobotOpMode clears the caches every loop; without
        // that clear, every read would return the same stale values forever.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.drivetrain = drivetrain;
        exampleSubsystem = profile == HardwareProfile.ALL ? new ExampleSubsystem(hardwareMap, telemetry) : null;
        limelight = profile != HardwareProfile.DRIVE_ONLY ? new Limelight(hardwareMap, drivetrain, telemetry) : null;
    }

    /** Invalidate all hubs' bulk sensor caches — called once per loop, before the scheduler tick. */
    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /** Permanent control updates run once per cycle, after Ivy has advanced actions. */
    public void periodic() {
        if (stopped) return;
        drivetrain.periodic();
        if (exampleSubsystem != null) exampleSubsystem.periodic();
        if (limelight != null) limelight.periodic();
    }

    public void updateTelemetry(double loopTimeMs) {
        telemetry.addData("Robot Profile", profile);
        if (Double.isFinite(loopTimeMs)) telemetry.addData("Loop Time (ms)", "%.1f", loopTimeMs);
        telemetry.update();
    }

    /** Try every cleanup even if one device fails during shutdown. */
    public void stop() {
        if (stopped) return;
        stopped = true;
        try {
            drivetrain.stop();
        } finally {
            try {
                if (exampleSubsystem != null) exampleSubsystem.stop();
            } finally {
                if (limelight != null) limelight.stop();
            }
        }
    }
}
