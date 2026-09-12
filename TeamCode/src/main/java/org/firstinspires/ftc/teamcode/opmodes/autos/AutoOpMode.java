package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

/**
 * Base class for all autos (spec §7.3). A red/blue pair is two tiny subclasses of
 * one auto class — geometry is authored ONCE in the red frame and mirrored here,
 * so a fix lands on both alliances by construction (DECODE's separate red/blue
 * files drifted apart; see the archived repo).
 *
 * Game-specific helpers (shoot(), aimForPath(), ...) get added HERE next season,
 * once they exist — never re-declared per auto.
 */
public abstract class AutoOpMode extends RobotOpMode {
    protected final Alliance alliance;
    private Command activeSequence;
    private boolean aborted;
    private String abortReason;

    protected AutoOpMode(Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * A red-frame point, mirrored automatically for blue. The mirrored heading is
     * meaningless here (2-arg Pose) — always set heading via transformedHeading().
     */
    protected Pose transformed(double x, double y) {
        Pose pose = new Pose(x, y);
        return alliance == Alliance.RED ? pose : mirror(pose);
    }

    /** A red-frame heading in degrees, mirrored for blue, returned in radians. */
    protected double transformedHeading(double degrees) {
        return Math.toRadians(alliance == Alliance.RED ? degrees : 180 - degrees);
    }

    /** The auto's starting pose (use transformed()/transformedHeading()). */
    protected abstract Pose startingPose();

    /** The whole auto as one Command (conventionally sequential(...)); built at start(). */
    protected abstract Command buildSequence();

    @Override
    protected final void onInit() {
        aborted = false;
        activeSequence = null;
        Alliance.current = alliance;
        robot.drivetrain.setStartingPose(startingPose());
        onAutoInit();
    }

    /** Build paths or initialize mechanisms after the alliance and starting pose are set. */
    protected void onAutoInit() { }

    @Override
    protected final void onStart() {
        if (!robot.drivetrain.isLocalizationReady()) { abortForLocalization(); return; }
        activeSequence = buildSequence();
        schedule(activeSequence);
    }

    @Override
    protected final void onInitLoop() {
        robot.telemetry.addData("Auto Pinpoint", robot.drivetrain.getLocalizationStatus());
        robot.telemetry.addLine("Keep robot still; press X in INIT to recalibrate Pinpoint. Wait for READY.");
        if (gamepad1.xWasPressed()) robot.drivetrain.recalibrateLocalization();
        onAutoInitLoop();
    }

    protected void onAutoInitLoop() { }

    @Override
    protected final void onLoop() {
        // Runs before Scheduler.execute: a stopped follower must not advance the sequence.
        if (!aborted && !robot.drivetrain.isLocalizationReady()) abortForLocalization();
        if (aborted) robot.telemetry.addData("Auto aborted - press STOP", abortReason);
        else onAutoLoop();
    }

    protected void onAutoLoop() { }

    private void abortForLocalization() {
        aborted = true;
        abortReason = robot.drivetrain.getLocalizationStatus();
        if (activeSequence != null) activeSequence.cancel();
        robot.stop();
        robot.telemetry.addData("Auto aborted", robot.drivetrain.getLocalizationStatus());
    }

    @Override
    protected final void onStop() {
        if (activeSequence != null) activeSequence.cancel();
    }
}
