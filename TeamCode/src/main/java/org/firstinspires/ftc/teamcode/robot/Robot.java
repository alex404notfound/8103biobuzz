package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/** NextFTC discovers this container once; hardware is acquired afresh at each INIT. */
public class Robot implements NextRobot {
    public enum HardwareProfile { DRIVE_ONLY, DRIVE_AND_VISION, ALL }

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Drivetrain drivetrain;
    public ExampleSubsystem exampleSubsystem;
    public Limelight limelight;
    public HardwareProfile profile;
    private List<LynxModule> hubs = Collections.emptyList();
    private Set<Mechanism> mechanisms = Collections.emptySet();
    private boolean stopped = true;
    private Runnable afterMechanisms = () -> { };

    /** Required by NextFTC's scanner. Do not access hardware during app startup. */
    public Robot() { }

    public void initialize(HardwareMap hardwareMap, Telemetry driverTelemetry, HardwareProfile profile) {
        stop();
        Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
        // NextFTC flushes both backends at the end of the loop. addBackend deduplicates
        // the Dashboard singleton; do not register a new MultipleTelemetry each INIT.
        dev.nextftc.robot.Telemetry.addBackend(dashboard);
        beginInitialization(hardwareMap, new MultipleTelemetry(driverTelemetry, dashboard), profile);
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        initializeOptionalMechanisms();
    }

    Robot(HardwareMap hardwareMap, Telemetry telemetry, HardwareProfile profile, Drivetrain drivetrain) {
        initialize(hardwareMap, telemetry, profile, drivetrain);
    }

    void initialize(HardwareMap hardwareMap, Telemetry telemetry, HardwareProfile profile, Drivetrain drivetrain) {
        stop();
        beginInitialization(hardwareMap, telemetry, profile);
        this.drivetrain = drivetrain;
        initializeOptionalMechanisms();
    }

    private void beginInitialization(HardwareMap hardwareMap, Telemetry telemetry, HardwareProfile profile) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.profile = profile;
        drivetrain = null;
        exampleSubsystem = null;
        limelight = null;
        mechanisms = Collections.emptySet();
        afterMechanisms = () -> { };
        stopped = false;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    private void initializeOptionalMechanisms() {
        exampleSubsystem = profile == HardwareProfile.ALL ? new ExampleSubsystem(hardwareMap, telemetry) : null;
        limelight = profile != HardwareProfile.DRIVE_ONLY ? new Limelight(hardwareMap, drivetrain, telemetry) : null;
        Set<Mechanism> ordered = new LinkedHashSet<>();
        ordered.add(drivetrain);
        if (exampleSubsystem != null) ordered.add(exampleSubsystem);
        if (limelight != null) ordered.add(limelight);
        // NextFTC updates mechanisms before its scheduler. Run the current-sample
        // autonomous fault guard last, before a stopped path could advance a sequence.
        ordered.add(new Mechanism() {
            @Override public void periodic() { if (!stopped) afterMechanisms.run(); }
        });
        mechanisms = Collections.unmodifiableSet(ordered);
    }

    @Override public Set<Mechanism> getMechanisms() {
        return stopped ? Collections.emptySet() : mechanisms;
    }

    public void setAfterMechanisms(Runnable callback) { afterMechanisms = callback; }

    /** NextFTC does not tick mechanisms in INIT; sample them explicitly in that phase. */
    public void updateMechanismsInInit() {
        for (Mechanism mechanism : getMechanisms()) mechanism.periodic();
    }

    /** Invalidate sensors before both INIT and active loops. */
    public void clearBulkCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    /** Add loop metrics; NextFTC's telemetry hook owns the flush. */
    public void updateTelemetry(double loopTimeMs) {
        telemetry.addData("Robot Profile", profile);
        if (Double.isFinite(loopTimeMs)) telemetry.addData("Loop Time (ms)", "%.1f", loopTimeMs);
    }

    /** Try every cleanup even if construction or another device's shutdown fails. */
    public void stop() {
        if (stopped) return;
        stopped = true;
        afterMechanisms = () -> { };
        try {
            if (drivetrain != null) drivetrain.stop();
        } finally {
            try {
                if (exampleSubsystem != null) exampleSubsystem.stop();
            } finally {
                if (limelight != null) limelight.stop();
            }
        }
    }
}
