package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.math.EncoderRpmEstimator;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;

/** Immediate motor power control, with rolling encoder speed and optional observed-turn RPM calibration. */
@Configurable
@TeleOp(name = "Motor RPM Test", group = "Diagnostics")
public class MotorRpmTest extends OpMode {
    public static volatile double power = 0;
    public static volatile int calibrationTurns = 10;
    public static volatile boolean markStart, markEnd, clearCalibration;
    public static volatile double measurementSeconds = 3.0;
    public static volatile double settleSeconds = 1.0;

    private DcMotorEx motor;
    private LynxModule controlHub;
    private EncoderRpmEstimator estimator;
    private double countsPerRotation, minimumTrialScale, maximumTrialScale;
    private double activeWindowSeconds, lastAppliedPower, lastTrialDifferencePercent;
    private long totalCalibrationCounts, totalCalibrationTurns;
    private int calibrationTrials, turnsInTrial;
    private int previousPosition;
    private long unwrappedPosition, calibrationStartPosition, sampleAfterNs;
    private boolean running, hasPosition, calibrating;
    private String calibrationStatus;

    @Override public void init() {
        resetControls();
        running = hasPosition = false;
        resetCalibration();
        activeWindowSeconds = validWindowSeconds();
        estimator = new EncoderRpmEstimator(activeWindowSeconds);
        try {
            telemetry = withBrowserTelemetry(telemetry);
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                if (hub.getSerialNumber().isEmbedded() && hub.isParent()) {
                    controlHub = hub;
                    break;
                }
            }
            if (controlHub == null) throw new IllegalStateException("Control Hub not found.");
            for (DcMotorEx candidate : hardwareMap.getAll(DcMotorEx.class)) {
                // Controller connection info includes the module address, excluding Expansion Hubs.
                if (candidate.getPortNumber() == 0 && candidate.getController().getConnectionInfo()
                        .equals(controlHub.getConnectionInfo())) {
                    motor = candidate;
                    break;
                }
            }
            if (motor == null) throw new IllegalStateException(
                    "Add a motor on Control Hub port 0 in the robot configuration; any name is OK.");
            MotorSafety.stopAndBrake(motor);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("Motor", hardwareMap.getNamesOf(motor));
            telemetry.addLine("Connect the encoder to Control Hub encoder port 0.");
            telemetry.addLine("START, then Panels > Configurables > MotorRpmTest.");
            telemetry.addLine("Set power from 0 to 1 after START. Calibration is optional; the motor runs immediately.");
            telemetry.addLine("Before calibration, speed is encoder counts/second; no motor type or rated RPM is assumed.");
            telemetry.addLine("For RPM: run slowly enough to count shaft turns, markStart=true, count calibrationTurns, markEnd=true.");
            telemetry.addLine("The motor keeps running during calibration. Default: 10 observed turns; no hand turning required.");
            telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void start() {
        try {
            resetControls();
            motor.setPower(0);
            hasPosition = false;
            lastAppliedPower = 0;
            sampleAfterNs = nowNs();
            estimator.reset();
            running = true;
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void loop() {
        if (!running) return;
        try {
            boolean begin = markStart, finish = markEnd, clear = clearCalibration;
            if (begin) markStart = false;
            if (finish) markEnd = false;
            if (clear) clearCalibration = false;
            double requestedPower = power;
            double appliedPower = Double.isFinite(requestedPower)
                    ? Math.max(0, Math.min(1, requestedPower)) : 0;
            motor.setPower(appliedPower);

            // Every read is fresh. Bracket the transaction to reduce variable read-latency bias.
            long beforeRead = nowNs();
            LynxModule.BulkData data = controlHub.getBulkData();
            long afterRead = nowNs();
            if (data.isFake()) throw new IllegalStateException("Control Hub did not return a real encoder sample.");
            int position = data.getMotorCurrentPosition(0);
            long timestamp = beforeRead + (afterRead - beforeRead) / 2;
            if (hasPosition) unwrappedPosition += position - previousPosition;
            else { unwrappedPosition = position; hasPosition = true; }
            previousPosition = position;

            double window = validWindowSeconds();
            if (window != activeWindowSeconds) {
                activeWindowSeconds = window;
                estimator = new EncoderRpmEstimator(window);
            }
            if (appliedPower != lastAppliedPower) {
                estimator.reset();
                sampleAfterNs = timestamp + (long) (validSettleSeconds() * 1_000_000_000L);
                lastAppliedPower = appliedPower;
            }
            if (clear) {
                resetCalibration();
                estimator.reset();
            } else if (begin) {
                estimator.reset();
                calibrating = false;
                int turns = calibrationTurns;
                if (turns >= 1 && turns <= 1000) {
                    turnsInTrial = turns;
                    calibrationStartPosition = unwrappedPosition;
                    calibrating = true;
                    calibrationStatus = "Count " + turnsInTrial + " complete shaft turns, then markEnd=true. Motor power stays active.";
                } else calibrationStatus = "Set calibrationTurns to a whole number from 1 to 1000, then markStart.";
            } else if (finish) {
                finishCalibration();
                estimator.reset();
                sampleAfterNs = timestamp;
            } else if (timestamp >= sampleAfterNs) {
                estimator.addSample(timestamp, position);
            }
            display(appliedPower, timestamp);
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void finishCalibration() {
        if (!calibrating) {
            calibrationStatus = "Set markStart before marking the end of a trial.";
            return;
        }
        long counts = Math.abs(unwrappedPosition - calibrationStartPosition);
        if (counts == 0) {
            calibrationStatus = "No movement measured. Check encoder wiring and power; count shaft turns, then markEnd.";
            return;
        }
        double trialScale = (double) counts / turnsInTrial;
        lastTrialDifferencePercent = countsPerRotation > 0
                ? 100 * Math.abs(trialScale - countsPerRotation) / countsPerRotation : Double.NaN;
        if (lastTrialDifferencePercent > 1) {
            calibrating = false;
            calibrationStatus = "Trial disagrees by over 1%: not added. Check turns/alignment, then repeat or clearCalibration.";
            return;
        }
        if (calibrationTrials == 0) minimumTrialScale = maximumTrialScale = trialScale;
        else {
            minimumTrialScale = Math.min(minimumTrialScale, trialScale);
            maximumTrialScale = Math.max(maximumTrialScale, trialScale);
        }
        totalCalibrationCounts += counts;
        totalCalibrationTurns += turnsInTrial;
        countsPerRotation = (double) totalCalibrationCounts / totalCalibrationTurns;
        calibrationTrials++;
        calibrating = false;
        calibrationStatus = "Ready. Repeat markStart/markEnd to refine calibration, or adjust power.";
    }

    private void display(double appliedPower, long timestamp) {
        telemetry.addData("Calibration", calibrationStatus);
        telemetry.addData("Calibration trials / total turns", "%d / %d", calibrationTrials, totalCalibrationTurns);
        if (Double.isFinite(lastTrialDifferencePercent)) {
            telemetry.addData("Last calibration trial difference (%)", "%.3f", lastTrialDifferencePercent);
        }
        if (totalCalibrationCounts > 0) {
            telemetry.addData("Calibration scale change per encoder count (%)", "%.3f", 100.0 / totalCalibrationCounts);
        }
        if (calibrationTrials > 1) {
            double spread = 100 * (maximumTrialScale - minimumTrialScale) / countsPerRotation;
            telemetry.addData("Calibration trial spread (%)", "%.3f", spread);
            if (spread > 1) telemetry.addLine("Trials disagree: check turn count and marker alignment; clearCalibration to redo.");
        }
        telemetry.addData("Applied power (0 to 1)", "%.2f", appliedPower);
        if (timestamp < sampleAfterNs) {
            telemetry.addLine("Waiting for the motor to settle after the power change...");
        } else if (estimator.isReady()) {
            telemetry.addData("Encoder speed (counts/second)", "%.1f", Math.abs(estimator.getCountsPerSecond()));
            if (!calibrating && countsPerRotation > 0) {
                double rpm = Math.abs(estimator.getCountsPerSecond()) * 60 / countsPerRotation;
                telemetry.addData("Measured RPM", "%.1f", rpm);
            }
            telemetry.addData("Samples / measured seconds", "%d / %.2f", estimator.getSampleCount(), estimator.getSpanSeconds());
        } else {
            telemetry.addData("Collecting measurement (seconds)", "%.2f / %.2f", estimator.getSpanSeconds(), activeWindowSeconds);
        }
        if (countsPerRotation <= 0 || calibrating) {
            telemetry.addLine("RPM needs an observed-turn calibration; power works without it.");
        }
        telemetry.addLine("Panels: power 0 to 1; measurementSeconds 1 to 10; settleSeconds 0 to 10.");
        telemetry.update();
    }

    @Override public void stop() {
        running = false;
        resetControls();
        MotorSafety.stopAndBrake(motor);
    }

    private static void resetControls() {
        power = 0;
        markStart = markEnd = clearCalibration = false;
    }

    private void resetCalibration() {
        countsPerRotation = 0;
        totalCalibrationCounts = totalCalibrationTurns = 0;
        calibrationTrials = 0;
        lastTrialDifferencePercent = Double.NaN;
        calibrating = false;
        calibrationStatus = "Optional for RPM: run slowly, markStart, count 10 turns (default), then markEnd.";
    }

    private static double validWindowSeconds() {
        double value = measurementSeconds;
        return Double.isFinite(value) ? Math.max(1, Math.min(10, value)) : 3;
    }

    private static double validSettleSeconds() {
        double value = settleSeconds;
        return Double.isFinite(value) ? Math.max(0, Math.min(10, value)) : 1;
    }

    protected long nowNs() { return System.nanoTime(); }

    protected Telemetry withBrowserTelemetry(Telemetry driver) {
        return RobotTelemetry.forOpMode(driver);
    }

    private void shutdown(RuntimeException failure) {
        try { stop(); }
        catch (RuntimeException stopFailure) {
            if (failure != stopFailure) failure.addSuppressed(stopFailure);
        }
        throw failure;
    }
}
