package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.LongSupplier;

import static com.pedropathing.api.Paths.line;

/**
 * One held practice approach: Pedro path to standoff, then live camera steering to a stop.
 * Adapted from Hive Vision's BallChaseFollower concept; no search sweep, intake or blind coasting.
 * One outer Ivy command owns the drive, including its delegated followPath command.
 */
public final class HivePedroController {
    public enum Phase { IDLE, TRAVEL, CHASE, REACHED, ABORTED }
    private final Drivetrain drive;
    private final HiveBallTracker tracker;
    private final LongSupplier clock;
    private final double approach, stopDistance, distanceTolerance, angleTolerance, forwardGain, turnGain;
    private final double maxPower, pathSpeed, acceleration, timeoutSeconds, maxTravel, maxTargetShift;
    private Command action, pathAction;
    private Phase phase = Phase.IDLE;
    private String status = "INIT: stationary camera check";
    private boolean enabled, held, releaseRequired = true, limitApplied, endHandled;
    private long stationarySince = -1, lastStationarySample, startedAt, chosenTrack;
    private double originalPowerLimit, traveled, ballX, ballY;
    private Pose previousPose, plannedEnd;

    public HivePedroController(Drivetrain drive, HiveBallTracker tracker) { this(drive, tracker, System::nanoTime); }

    HivePedroController(Drivetrain drive, HiveBallTracker tracker, LongSupplier clock) {
        this.drive = drive;
        this.tracker = tracker;
        this.clock = clock;
        approach = HivePedroConfig.approachDistanceInches;
        stopDistance = HivePedroConfig.stopDistanceInches;
        distanceTolerance = HivePedroConfig.distanceToleranceInches;
        angleTolerance = HivePedroConfig.angleToleranceDegrees;
        forwardGain = HivePedroConfig.forwardGain;
        turnGain = HivePedroConfig.turnGain;
        maxPower = HivePedroConfig.maximumMotorPower;
        pathSpeed = HivePedroConfig.maximumPathSpeedInchesPerSecond;
        acceleration = HivePedroConfig.maximumPathAcceleration;
        timeoutSeconds = HivePedroConfig.maximumAttemptSeconds;
        maxTravel = HivePedroConfig.maximumTravelInches;
        maxTargetShift = HivePedroConfig.maximumTargetShiftInches;
        for (double value : new double[]{approach, stopDistance, distanceTolerance, angleTolerance,
                forwardGain, turnGain, maxPower, pathSpeed, acceleration, timeoutSeconds, maxTravel, maxTargetShift}) {
            if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException("Hive Pedro settings must be finite and positive");
        }
        if (approach <= stopDistance + distanceTolerance || distanceTolerance >= stopDistance
                || angleTolerance >= 15 || maxPower > 0.4 || pathSpeed > 20 || timeoutSeconds > 30)
            throw new IllegalArgumentException("Invalid Hive Pedro distances or practice limits (power <= .4, speed <= 20, time <= 30)");
    }

    public void start() {
        enabled = true;
        releaseRequired = true;
        stationarySince = -1;
        status = "Release RT and center sticks, then hold RT to approach";
    }

    /** Call with the new camera sample BEFORE Drivetrain.periodic can issue another path output. */
    public void beforeDrive(boolean holdTrigger, boolean cancel) {
        if (!enabled) return;
        held = holdTrigger;
        if (cancel) { abort("Cancelled; release RT to retry"); releaseRequired = true; return; }
        if (!held) {
            if (isActive()) abort("RT released");
            releaseRequired = false;
            return;
        }
        if (isActive()) guard();
    }

    /** After the current Pinpoint sample, before NextFTC executes Ivy. */
    public void afterSensors() {
        if (!enabled) return;
        if (isActive()) { guard(); return; }
        observeStationary();
        if (!held || releaseRequired) return;
        String readiness = readiness();
        if (readiness != null) { status = readiness; return; }
        beginAttempt();
    }

    private void observeStationary() {
        Velocity velocity = drive.getVelocity();
        long now = clock.getAsLong();
        if (!drive.isLocalizationReady() || !Constants.isFiniteVelocity(velocity)
                || Math.hypot(velocity.vx, velocity.vy) > 1 || Math.abs(velocity.omega) > Math.toRadians(10)) {
            stationarySince = -1;
        } else if (stationarySince < 0 || now - lastStationarySample < 0 || now - lastStationarySample > 100_000_000L)
            stationarySince = now;
        lastStationarySample = now;
    }

    private String readiness() {
        if (!drive.isPathFollowingConfigured()) return "Run Pedro AutoTune and configure Foresight first";
        if (!drive.isLocalizationReady()) return "Pinpoint is not READY";
        if (!tracker.hasGeometry()) return "Measure and verify HiveVisionConfig camera geometry first";
        if (stationarySince < 0 || (clock.getAsLong() - stationarySince) / 1e6 < 250)
            return "Keep still for 250 ms before acquiring the approach";
        HiveBallTracker.Detection target = tracker.getTarget();
        if (target == null || target.position == null) return "Waiting for a confirmed floor ball";
        if (target.position.forwardInches <= 0) return "Target must be in front of the robot";
        double range = Math.hypot(target.position.forwardInches, target.position.leftInches);
        if (range - stopDistance > maxTravel) return "Ball exceeds the configured travel budget";
        return null;
    }

    private void beginAttempt() {
        HiveBallTracker.Detection target = tracker.getTarget();
        Pose start = drive.getPose();
        if (target == null || target.position == null || !Constants.isFinitePose(start)) return;
        double[] ball = fieldPoint(start, target);
        ballX = ball[0]; ballY = ball[1];
        chosenTrack = tracker.getTrackId();
        previousPose = start;
        traveled = 0;
        startedAt = clock.getAsLong();
        endHandled = false;
        stationarySince = -1;
        double range = Math.hypot(ballX - start.x(), ballY - start.y());
        double heading = start.heading() + wrap(Math.atan2(ballY - start.y(), ballX - start.x()) - start.heading());
        plannedEnd = new Pose(ballX - approach * Math.cos(heading), ballY - approach * Math.sin(heading), heading);
        final Path path = range > approach + 4 ? line(start, plannedEnd).linear(start.heading(), heading).with(
                Constants.foresightConfig.maxVelocityConstraint.at(pathSpeed),
                Constants.foresightConfig.maxAccelerationConstraint.at(acceleration)) : null;
        action = Command.build().requiring(drive)
                .setStart(() -> {
                    originalPowerLimit = drive.getMotorPowerLimit();
                    drive.setMotorPowerLimit(Math.min(originalPowerLimit, maxPower));
                    limitApplied = true;
                    if (path != null) {
                        phase = Phase.TRAVEL;
                        status = "Pedro approach";
                        pathAction = drive.followPath(path);
                        pathAction.start();
                    } else enterChase();
                })
                .setExecute(this::execute)
                .setDone(() -> phase == Phase.REACHED || phase == Phase.ABORTED)
                .setEnd(this::endAttempt);
        try { action.schedule(); }
        catch (RuntimeException failure) {
            try { endAttempt(EndCondition.INTERRUPTED); }
            catch (RuntimeException cleanupFailure) { failure.addSuppressed(cleanupFailure); }
            throw failure;
        }
    }

    private void execute() {
        if (!guard()) return;
        if (phase == Phase.TRAVEL) {
            if (pathAction.done()) {
                pathAction.end(EndCondition.NATURALLY);
                pathAction = null;
                enterChase();
            } else pathAction.execute();
        } else if (phase == Phase.CHASE) {
            HiveBallTracker.Detection target = tracker.getTarget();
            if (target == null || target.position == null) { abort("Target lost"); return; }
            double forward = target.position.forwardInches, left = target.position.leftInches;
            double range = Math.hypot(forward, left);
            double bearing = Math.toDegrees(Math.atan2(left, forward));
            if (range <= stopDistance + distanceTolerance && Math.abs(bearing) <= angleTolerance) {
                phase = Phase.REACHED;
                releaseRequired = true;
                status = "At test standoff; release RT before another attempt";
                drive.stop();
                return;
            }
            // Do not reverse toward an old observation or drive forward while pointed far off target.
            double fwd = Math.abs(bearing) < 15 ? Math.max(0, (range - stopDistance) * forwardGain) : 0;
            double turn = bearing * turnGain;
            drive.driveRobotRelative(Math.min(maxPower, fwd), 0, Math.max(-maxPower, Math.min(maxPower, turn)));
        }
    }

    private void enterChase() { drive.stopFollowing(); phase = Phase.CHASE; status = "Camera final approach through Pedro drive"; }

    private boolean guard() {
        if (!isActive()) return false;
        if (!held) { abort("RT released"); return false; }
        if (!drive.isLocalizationReady() || !Constants.isFinitePose(drive.getPose())) {
            abort("Pinpoint lost: " + drive.getLocalizationStatus()); return false;
        }
        if (!drive.isPathFollowingConfigured()) { abort("Pedro tuning gate disabled"); return false; }
        if ((clock.getAsLong() - startedAt) / 1e9 >= timeoutSeconds) { abort("Attempt timed out"); return false; }
        Pose pose = drive.getPose();
        traveled += Math.hypot(pose.x() - previousPose.x(), pose.y() - previousPose.y());
        previousPose = pose;
        if (traveled > maxTravel) { abort("Travel budget reached"); return false; }
        HiveBallTracker.Detection target = tracker.getTarget();
        if (target == null || target.position == null || target.position.forwardInches <= 0) {
            abort("Ball lost, stale, or outside calibrated floor area"); return false;
        }
        if (tracker.getTrackId() != chosenTrack) { abort("Target changed; release RT to reacquire"); return false; }
        double[] point = fieldPoint(pose, target);
        if (Math.hypot(point[0] - ballX, point[1] - ballY) > maxTargetShift) {
            abort("Ball position shifted; release RT to reacquire"); return false;
        }
        return true;
    }

    public void abort(String reason) {
        if (isActive()) {
            phase = Phase.ABORTED;
            status = reason;
            if (action != null) action.cancel();
            drive.stop();
        } else status = reason;
        releaseRequired = true;
        stationarySince = -1;
    }

    private void endAttempt(EndCondition condition) {
        if (endHandled) return;
        endHandled = true;
        try {
            if (pathAction != null) { pathAction.end(condition); pathAction = null; }
        } finally {
            try { drive.stop(); }
            finally {
                if (limitApplied) { drive.setMotorPowerLimit(originalPowerLimit); limitApplied = false; }
                if (phase != Phase.REACHED && phase != Phase.ABORTED) { phase = Phase.ABORTED; status = "Drive command interrupted"; }
                releaseRequired = true;
            }
        }
    }

    public void stop() { abort("Stopped"); enabled = false; drive.stop(); }
    public boolean isActive() { return phase == Phase.TRAVEL || phase == Phase.CHASE; }
    public Phase getPhase() { return phase; }
    public String getStatus() { return status; }
    public Pose getPlannedEnd() { return plannedEnd; }
    public double getTraveledInches() { return traveled; }

    private static double[] fieldPoint(Pose pose, HiveBallTracker.Detection target) {
        double forward = target.position.forwardInches, left = target.position.leftInches;
        return new double[]{pose.x() + forward * Math.cos(pose.heading()) - left * Math.sin(pose.heading()),
                pose.y() + forward * Math.sin(pose.heading()) + left * Math.cos(pose.heading())};
    }
    private static double wrap(double radians) { return Math.atan2(Math.sin(radians), Math.cos(radians)); }
}
