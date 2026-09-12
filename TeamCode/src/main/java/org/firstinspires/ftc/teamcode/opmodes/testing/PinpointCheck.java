package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Sensor-only check: no drive motors, example mechanism, camera, or pose reset. */
@TeleOp(name = "Pinpoint Check", group = "Diagnostics")
public class PinpointCheck extends OpMode {
    private GoBildaPinpointDriver pinpoint;
    private boolean calibrationPending;
    private long calibrationStarted;
    private double configuredForwardOffset;
    private double configuredStrafeOffset;

    @Override public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConstants.hardwareMapName);
        pinpoint.setOffsets(Constants.localizerConstants.forwardPodY, Constants.localizerConstants.strafePodX,
                Constants.localizerConstants.distanceUnit);
        pinpoint.setEncoderDirections(Constants.localizerConstants.forwardEncoderDirection,
                Constants.localizerConstants.strafeEncoderDirection);
        if (Constants.localizerConstants.customEncoderResolution.isPresent())
            pinpoint.setEncoderResolution(Constants.localizerConstants.customEncoderResolution.getAsDouble(),
                    Constants.localizerConstants.distanceUnit);
        else pinpoint.setEncoderResolution(Constants.localizerConstants.encoderResolution);
        if (Constants.localizerConstants.yawScalar.isPresent())
            pinpoint.setYawScalar(Constants.localizerConstants.yawScalar.getAsDouble());
        configuredForwardOffset = pinpoint.getXOffset(DistanceUnit.INCH);
        configuredStrafeOffset = pinpoint.getYOffset(DistanceUnit.INCH);
        telemetry.addLine("Keep robot stationary. In INIT, press X to recalibrate gyro; wait for READY.");
        telemetry.update();
    }

    @Override public void init_loop() {
        if (gamepad1.xWasPressed()) {
            pinpoint.recalibrateIMU();
            calibrationPending = true;
            calibrationStarted = System.nanoTime();
        }
        sampleAndDisplay();
        telemetry.addLine("Keep still; X recalibrates gyro without resetting field pose.");
        telemetry.addLine("After READY, press START and push forward, left, then rotate counterclockwise.");
        telemetry.update();
    }

    @Override public void loop() {
        sampleAndDisplay();
        telemetry.addLine("Forward: X increases at heading 0. Left: Y increases. Counterclockwise: heading increases.");
        telemetry.addLine("No motors are commanded. Return to INIT for stationary gyro calibration.");
        telemetry.update();
    }

    private void sampleAndDisplay() {
        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (calibrationPending && System.nanoTime() - calibrationStarted >= 300_000_000L
                && status == GoBildaPinpointDriver.DeviceStatus.READY) calibrationPending = false;
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pinpoint Status", status);
        telemetry.addData("Calibration", calibrationPending ? "KEEP STILL" : "No calibration pending");
        telemetry.addData("Pinpoint Version", pinpoint.getDeviceVersion());
        telemetry.addData("Pinpoint Frequency (Hz)", pinpoint.getFrequency());
        telemetry.addData("Configured pod resolution", Constants.localizerConstants.customEncoderResolution.isPresent()
                ? Constants.localizerConstants.customEncoderResolution.getAsDouble() + " ticks/" + Constants.localizerConstants.distanceUnit
                : Constants.localizerConstants.encoderResolution);
        telemetry.addData("Forward / strafe direction", Constants.localizerConstants.forwardEncoderDirection + " / "
                + Constants.localizerConstants.strafeEncoderDirection);
        telemetry.addData("Live forward / strafe offsets (in)", "%.3f / %.3f", configuredForwardOffset, configuredStrafeOffset);
        telemetry.addData("X / Y (in)", "%.3f / %.3f", pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (degrees)", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Forward / strafe ticks", "%d / %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
    }
}
