package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HiveTagAim;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.Collections;
import java.util.List;

/** Practice only. Bench mode needs just the launcher, servo and Limelight; drive is opt-in. */
@Config
@TeleOp(name = "Launcher Prototype", group = "Prototyping")
public class LauncherPrototype extends OpMode {
    public static boolean enableDrive = false;
    public static Alliance startingAlliance = Alliance.RED;
    public static int targetTagId = 34;
    public static double rpmStep = 100, hoodJogStep = 0.005;

    private Launcher launcher;
    private Limelight vision;
    private HiveTagAim aim;
    private Drivetrain drive;
    private Alliance alliance;
    private List<LynxModule> hubs = Collections.emptyList();
    private boolean active, stopped, cancelLatched;

    public LauncherPrototype() { }

    /** Components can also be supplied by a bench harness without creating FTC hardware. */
    LauncherPrototype(Launcher launcher, Limelight vision, HiveTagAim aim, Drivetrain drive,
                      Telemetry telemetry, Alliance alliance) {
        this.launcher = launcher;
        this.vision = vision;
        this.aim = aim;
        this.drive = drive;
        this.telemetry = telemetry;
        this.alliance = alliance;
    }

    @Override public void init() {
        Scheduler.reset();
        active = false;
        stopped = false;
        cancelLatched = false;
        alliance = startingAlliance == null ? Alliance.RED : startingAlliance;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            launcher = new Launcher(hardwareMap, telemetry);
            // Relative tag observations also work without Pinpoint or drivetrain hardware.
            vision = new Limelight(hardwareMap, telemetry);
            aim = new HiveTagAim(Limelight.pipelineIndex);
            if (enableDrive) drive = new Drivetrain(hardwareMap, telemetry);
        } catch (RuntimeException failure) {
            stop();
            throw failure;
        }
    }

    @Override public void init_loop() {
        if (stopped) return;
        if (gamepad2.leftBumperWasPressed()) { alliance = Alliance.RED; targetTagId = 34; }
        if (gamepad2.rightBumperWasPressed()) { alliance = Alliance.BLUE; targetTagId = 38; }
        if (drive != null && gamepad2.xWasPressed()) drive.recalibrateLocalization();
        updateTagSelection();
        sample();
        launcher.periodic(); // Disabled: sensor readout only; hood remains untouched.
        report();
        telemetry.addLine("INIT: gamepad2 LB/RB selects alliance; X calibrates Pinpoint if drive enabled.");
        telemetry.addLine("No hood position is written until a command after START.");
        telemetry.update();
    }

    @Override public void start() {
        if (stopped) return;
        active = true;
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
        launcher.enable();
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            updateTagSelection();
            sample();
            updateLauncher();
            launcher.periodic();
            updateDrive();
            report();
            telemetry.update();
        } catch (RuntimeException failure) {
            stop();
            throw failure;
        }
    }

    private void sample() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        if (drive != null) drive.periodic();
        vision.periodic();
        aim.update(vision.getFreshResult(), alliance, targetTagId);
    }

    private void updateLauncher() {
        boolean cancel = gamepad1.x && !gamepad1.left_bumper;
        if (cancel) cancelLatched = true;
        if (cancelLatched) {
            launcher.idle();
            // Do not resume a previously held run/aim request when the cancel button is released.
            if (controlsReleased()) cancelLatched = false;
            return;
        }
        boolean testLeft = gamepad1.left_bumper && gamepad1.x && !gamepad1.y;
        boolean testRight = gamepad1.left_bumper && gamepad1.y && !gamepad1.x;
        if (testLeft || testRight) launcher.requestMotorTest(testLeft);
        else if (!gamepad1.left_bumper && gamepad1.right_trigger > 0.25) launcher.requestVelocity();
        else launcher.idle();

        if (gamepad1.aWasPressed()) {
            launcher.selectBall(Launcher.Ball.SMALL_POLLEN);
            launcher.applyPresetHood();
        }
        if (gamepad1.bWasPressed()) {
            launcher.selectBall(Launcher.Ball.LARGE_NECTAR);
            launcher.applyPresetHood();
        }
        if (gamepad1.dpadUpWasPressed()) launcher.adjustRpm(rpmStep);
        if (gamepad1.dpadDownWasPressed()) launcher.adjustRpm(-rpmStep);
        if (!gamepad1.left_bumper && gamepad1.yWasPressed()) launcher.positionHood(Launcher.calibrationHood);
        if (Double.isFinite(hoodJogStep) && hoodJogStep > 0 && hoodJogStep <= 0.02) {
            if (gamepad1.dpadLeftWasPressed()) launcher.jogHood(-hoodJogStep);
            if (gamepad1.dpadRightWasPressed()) launcher.jogHood(hoodJogStep);
        }
    }

    private void updateDrive() {
        if (drive == null) return;
        if (cancelLatched || (gamepad1.x && !gamepad1.left_bumper)) { drive.stop(); return; }
        if (gamepad2.left_trigger > 0.25 && Math.abs(gamepad2.right_stick_x) < 0.1) {
            // No tag => zero turn. Manual turn overrides; aiming never changes field pose.
            drive.turnInPlace(aim.getTurnPower());
        } else {
            drive.arcadeDrive(-gamepad2.left_stick_y, gamepad2.left_stick_x,
                    gamepad2.right_stick_x, alliance);
        }
    }

    private boolean controlsReleased() {
        return gamepad1.right_trigger <= 0.25 && !gamepad1.left_bumper
                && !gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y
                && !gamepad1.dpad_left && !gamepad1.dpad_right
                && gamepad2.left_trigger <= 0.25
                && Math.abs(gamepad2.left_stick_x) < 0.1 && Math.abs(gamepad2.left_stick_y) < 0.1
                && Math.abs(gamepad2.right_stick_x) < 0.1;
    }

    private void updateTagSelection() {
        int first = alliance == Alliance.RED ? 30 : 38;
        if (gamepad2.dpadLeftWasPressed()) targetTagId = first + Math.floorMod(targetTagId - first - 1, 8);
        if (gamepad2.dpadRightWasPressed()) targetTagId = first + Math.floorMod(targetTagId - first + 1, 8);
    }

    private void report() {
        telemetry.addData("Prototype mode", drive == null ? "BENCH (no drivetrain)" : "DRIVE + LAUNCHER");
        if (cancelLatched) telemetry.addLine("Cancel latched: release launcher/aim inputs and center drive sticks.");
        telemetry.addData("Alliance (locked at START)", alliance);
        telemetry.addData("Selected tag / cell", targetTagId + " / " + HiveCell.fromTagId(targetTagId));
        telemetry.addData("Tag tx / ty (deg)", "%.2f / %.2f", aim.getTx(), aim.getTy());
        telemetry.addData("Tag error / turn", "%.2f / %.3f", aim.getErrorDegrees(), aim.getTurnPower());
        telemetry.addData("Camera-tag slant range (in)", aim.getRangeInches());
        telemetry.addData("Tag status", aim.getStatus());
        telemetry.addLine("GP1 RT: hold RPM | LB+X / LB+Y: test left/right | X: cancel");
        telemetry.addLine("GP1 A/B: small/big + hood | up/down: RPM | Y: calibration hood | left/right: jog");
        telemetry.addLine("GP2 left/right: tag ID | sticks: drive | LT: hold aim (drive mode)");
        telemetry.addLine("Practice only. Driver selects upward CELL; tag alignment is not a firing solution.");
    }

    @Override public void stop() {
        active = false;
        if (stopped) return;
        stopped = true;
        try {
            if (launcher != null) launcher.stop();
        } finally {
            try {
                if (drive != null) drive.stop();
            } finally {
                try { if (vision != null) vision.stop(); }
                finally { if (aim != null) aim.clear(); Scheduler.reset(); }
            }
        }
    }
}
