package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Canvas;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.telemetry.FieldMapTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.PanelsFieldTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

/** Standalone Pinpoint pose/orientation and Panels field test; acquires no other hardware. */
@Configurable
@TeleOp(name = "Pinpoint Check", group = "Diagnostics")
public class PinpointCheck extends OpMode {
    // Applied only by Cross in INIT. The default center is a test reference, not a field measurement.
    public static volatile double startX = 72, startY = 72, startHeadingDegrees = 0;

    private GoBildaPinpointDriver pinpoint;
    private FieldMapTelemetry fieldMap;
    private boolean calibrationPending;
    private boolean stopped;
    private long calibrationStarted;
    private double configuredForwardOffset;
    private double configuredStrafeOffset;
    private String poseStatus;

    @Override public void init() {
        stopped = calibrationPending = false;
        poseStatus = "Retaining Pinpoint pose; Cross in INIT applies the proposed starting pose";
        telemetry = withBrowserTelemetry(telemetry);
        telemetry.setMsTransmissionInterval(100);
        fieldMap = new FieldMapTelemetry(telemetry, this::publishField, this::nowNs);
        fieldMap.update(null, false, "Waiting for first Pinpoint sample");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        pinpoint.setOffsets(Constants.localizerConfig.xPodOffset.get(), Constants.localizerConfig.yPodOffset.get(),
                Constants.localizerConfig.offsetUnits.get());
        pinpoint.setEncoderDirections(Constants.localizerConfig.xPodDirection.get(),
                Constants.localizerConfig.yPodDirection.get());
        if (Constants.localizerConfig.ticksPerUnit.get().isPresent())
            pinpoint.setEncoderResolution(Constants.localizerConfig.ticksPerUnit.get().getAsDouble(),
                    Constants.localizerConfig.encoderResolutionUnit.get());
        else pinpoint.setEncoderResolution(Constants.localizerConfig.podType.get());
        configuredForwardOffset = pinpoint.getXOffset(DistanceUnit.INCH);
        configuredStrafeOffset = pinpoint.getYOffset(DistanceUnit.INCH);
        showInstructions();
        telemetry.update();
    }

    @Override public void init_loop() {
        sampleAndDisplay(true);
    }

    @Override public void loop() {
        sampleAndDisplay(false);
    }

    private void sampleAndDisplay(boolean inInit) {
        if (stopped) return;
        try {
            boolean calibrate = inInit && gamepad1.squareWasPressed();
            boolean applyStart = inInit && !calibrate && gamepad1.crossWasPressed();
            if (calibrate) {
                pinpoint.recalibrateIMU();
                calibrationPending = true;
                calibrationStarted = nowNs();
                fieldMap.resetTrail();
                poseStatus = "Keep still; gyro calibration preserves position";
            }
            sampleAndDisplayPose(applyStart);
            showInstructions();
            telemetry.update();
        } catch (RuntimeException failure) {
            stopped = true;
            fieldMap.resetTrail();
            fieldMap.update(null, false, "Pinpoint read failed: " + failure.getMessage());
            telemetry.update();
            throw failure;
        }
    }

    private void sampleAndDisplayPose(boolean applyStart) {
        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (calibrationPending && nowNs() - calibrationStarted >= 300_000_000L
                && status == GoBildaPinpointDriver.DeviceStatus.READY) calibrationPending = false;
        Pose2D pose = pinpoint.getPosition();
        Pose fieldPose = toFieldPose(pose);
        boolean ready = status == GoBildaPinpointDriver.DeviceStatus.READY
                && !calibrationPending && Constants.isFinitePose(fieldPose);
        if (applyStart) {
            double x = startX, y = startY, heading = startHeadingDegrees;
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)
                    || x < 0 || x > FieldConstants.FIELD_WIDTH_INCHES
                    || y < 0 || y > FieldConstants.FIELD_LENGTH_INCHES) {
                poseStatus = "Start rejected: X/Y must be within 0..144 inches; heading must be finite";
            } else if (!ready) {
                poseStatus = "Wait for READY and completed calibration, then press Cross again";
            } else {
                pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
                pinpoint.setPosition(pose);
                fieldPose = toFieldPose(pose);
                fieldMap.resetTrail();
                poseStatus = "Starting pose applied; use measured coordinates for actual field position";
            }
        }
        fieldMap.update(fieldPose, ready, calibrationPending ? "CALIBRATING - KEEP STILL"
                : !Constants.isFinitePose(fieldPose) ? "INVALID POSE" : String.valueOf(status));
        telemetry.addData("Pinpoint Status", status);
        telemetry.addData("Calibration", calibrationPending ? "KEEP STILL" : "No calibration pending");
        telemetry.addData("Starting pose", poseStatus);
        int version = pinpoint.getDeviceVersion();
        telemetry.addData("Pinpoint Version", version);
        telemetry.addData("Pinpoint Frequency (Hz)", pinpoint.getFrequency());
        telemetry.addData("Configured pod resolution", Constants.localizerConfig.ticksPerUnit.get().isPresent()
                ? Constants.localizerConfig.ticksPerUnit.get().getAsDouble() + " ticks/" + Constants.localizerConfig.encoderResolutionUnit.get()
                : Constants.localizerConfig.podType.get());
        telemetry.addData("Forward / strafe direction", Constants.localizerConfig.xPodDirection.get() + " / "
                + Constants.localizerConfig.yPodDirection.get());
        telemetry.addData("Live forward / strafe offsets (in)", "%.3f / %.3f", configuredForwardOffset, configuredStrafeOffset);
        // Never publish stale/non-finite measurements as a current pose or graph value.
        telemetry.addData("pinpoint.xInches", ready ? fieldPose.x() : "unavailable");
        telemetry.addData("pinpoint.yInches", ready ? fieldPose.y() : "unavailable");
        telemetry.addData("pinpoint.headingDegrees", ready ? Math.toDegrees(fieldPose.heading()) : "unavailable");
        telemetry.addData("pinpoint.headingRadians", ready ? fieldPose.heading() : "unavailable");
        if (version >= 3 && ready) {
            telemetry.addData("pinpoint.pitchDegrees", finiteOrUnavailable(pinpoint.getPitch(AngleUnit.DEGREES)));
            telemetry.addData("pinpoint.rollDegrees", finiteOrUnavailable(pinpoint.getRoll(AngleUnit.DEGREES)));
        } else {
            String tiltStatus = version >= 3 ? "unavailable" : "Not supported by this firmware";
            telemetry.addData("pinpoint.pitchDegrees", tiltStatus);
            telemetry.addData("pinpoint.rollDegrees", tiltStatus);
        }
        telemetry.addData("Forward / strafe ticks", "%d / %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
    }

    private void showInstructions() {
        telemetry.addLine("Pinpoint only. Move robot by hand; no motors or servos are acquired.");
        telemetry.addLine("Panels http://192.168.43.1:8001 > Telemetry + Field: live robot, heading arrow and trail.");
        telemetry.addLine("In INIT, keep still and press Square to calibrate. At READY, press Cross to apply the starting pose.");
        telemetry.addLine("Default start (72,72,0) is a test reference. Enter measured coordinates for a field pose.");
        telemetry.addLine("Field inches: origin bottom-left; 0 degrees right, 90 degrees up. Pose buttons work only in INIT.");
    }

    private static Pose toFieldPose(Pose2D pose) {
        return pose == null ? null : new Pose(pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.RADIANS));
    }

    private static Object finiteOrUnavailable(double value) {
        return Double.isFinite(value) ? value : "unavailable";
    }

    @Override public void stop() {
        if (stopped) return;
        stopped = true;
        if (fieldMap != null) {
            fieldMap.resetTrail();
            fieldMap.update(null, false, "Pinpoint Check stopped");
            telemetry.update();
        }
    }

    protected Telemetry withBrowserTelemetry(Telemetry driver) { return RobotTelemetry.forOpMode(driver); }
    protected void publishField(Canvas canvas) { PanelsFieldTelemetry.send(canvas); }
    protected long nowNs() { return System.nanoTime(); }
}
