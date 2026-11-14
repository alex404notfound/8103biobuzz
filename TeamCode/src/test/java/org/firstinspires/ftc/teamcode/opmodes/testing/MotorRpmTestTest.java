package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.*;

public class MotorRpmTestTest {
    private final LynxModule controlHub = mock(LynxModule.class);
    private final LynxModule expansionHub = mock(LynxModule.class);
    private final LynxModule.BulkData data = mock(LynxModule.BulkData.class);
    private final DcMotorEx motor = mock(DcMotorEx.class);
    private final DcMotorEx expansionMotor = mock(DcMotorEx.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private final AtomicLong now = new AtomicLong();
    private MotorRpmTest op;

    @Before public void setUp() {
        defaults();
        MotorRpmTest.measurementSeconds = 1;
        MotorRpmTest.settleSeconds = 0;
        HardwareMap map = mock(HardwareMap.class);
        SerialNumber embedded = mock(SerialNumber.class);
        when(embedded.isEmbedded()).thenReturn(true);
        when(controlHub.getSerialNumber()).thenReturn(embedded);
        when(expansionHub.getSerialNumber()).thenReturn(embedded);
        when(controlHub.isParent()).thenReturn(true);
        when(controlHub.getConnectionInfo()).thenReturn("embedded; module 173");
        when(controlHub.getBulkData()).thenReturn(data);
        DcMotorController controller = mock(DcMotorController.class);
        DcMotorController expansionController = mock(DcMotorController.class);
        when(controller.getConnectionInfo()).thenReturn("embedded; module 173");
        when(expansionController.getConnectionInfo()).thenReturn("embedded; module 1");
        when(motor.getController()).thenReturn(controller);
        when(expansionMotor.getController()).thenReturn(expansionController);
        when(map.getAll(LynxModule.class)).thenReturn(Arrays.asList(expansionHub, controlHub));
        when(map.getAll(DcMotorEx.class)).thenReturn(Arrays.asList(expansionMotor, motor));
        when(map.getNamesOf(motor)).thenReturn(Collections.singleton("anyConfiguredName"));
        op = new MotorRpmTest() {
            @Override protected long nowNs() { return now.get(); }
            @Override protected Telemetry withBrowserTelemetry(Telemetry driver) { return driver; }
        };
        op.hardwareMap = map;
        op.telemetry = telemetry;
        MotorRpmTest.power = 1;
        op.init();
    }

    @After public void tearDown() { defaults(); }

    private static void defaults() {
        MotorRpmTest.power = 0;
        MotorRpmTest.calibrationTurns = 10;
        MotorRpmTest.markStart = MotorRpmTest.markEnd = MotorRpmTest.clearCalibration = false;
        MotorRpmTest.measurementSeconds = 3;
        MotorRpmTest.settleSeconds = 1;
    }

    @Test public void selectsControlHubPortZeroAndStartsStoppedDespiteInitPowerEdits() {
        assertEquals(0, MotorRpmTest.power, 0);
        verify(motor).setPower(0);
        verify(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(expansionMotor, never()).setPower(anyDouble());
        MotorRpmTest.power = 0.8;
        op.start(); op.loop();
        assertEquals(0, MotorRpmTest.power, 0);
        verify(motor, never()).setPower(0.8);
    }

    @Test public void tenTurnCalibrationAndRollingFitMeasureRpmWithoutMotorType() {
        op.start();
        calibrateAt(0, 280);
        clearInvocations(telemetry);
        for (int i = 0; i < 4; i++) sample(i * 250_000_000L, i * 7);
        verify(telemetry, never()).addData(eq("Measured RPM"), anyString(), anyDouble());
        sample(1_000_000_000L, 28);
        verify(telemetry).addData("Measured RPM", "%.1f", 60.0);
        verify(motor, never()).getVelocity();
        verify(motor, never()).getMotorType();
        verify(motor, never()).getCurrentPosition();
    }

    @Test public void keepsFractionalCountsPerTurnInsteadOfRoundingAnUnknownGearRatio() {
        op.start();
        calibrateAt(0, 285);
        sample(0, 0);
        sample(500_000_000L, 28);
        sample(1_000_000_000L, 57);
        verify(telemetry).addData("Measured RPM", "%.1f", 120.0);
    }

    @Test public void repeatingCalibrationCombinesTrialsAndRejectsAMiscount() {
        op.start();
        calibrateAt(0, 280);
        calibrateAt(280, 280);
        verify(telemetry).addData("Calibration trials / total turns", "%d / %d", 2, 20L);
        clearInvocations(telemetry);
        calibrateAt(560, 560); // Wrong turn count would imply a scale twice as large.
        verify(telemetry, never()).addData("Calibration trials / total turns", "%d / %d", 3, 30L);
        verify(telemetry).addData("Last calibration trial difference (%)", "%.3f", 100.0);
    }

    @Test public void panelsPowerIsBoundedAndInvalidNumbersStopTheMotor() {
        op.start();
        clearInvocations(motor);
        for (double value : new double[]{0.37, 2, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            MotorRpmTest.power = value;
            op.loop();
        }
        verify(motor).setPower(0.37);
        verify(motor).setPower(1);
        verify(motor, times(3)).setPower(0);
    }

    @Test public void powerChangesDiscardOldSpeedAndWaitForSettlingAndFullWindow() {
        op.start();
        calibrateAt(0, 280);
        for (int i = 0; i <= 4; i++) sample(i * 250_000_000L, i * 7);
        verify(telemetry).addData("Measured RPM", "%.1f", 60.0);
        clearInvocations(telemetry);
        MotorRpmTest.settleSeconds = 0.5;
        MotorRpmTest.power = 0.5;
        sample(1_250_000_000L, 35);
        sample(1_500_000_000L, 42);
        verify(telemetry, never()).addData(eq("Measured RPM"), anyString(), anyDouble());
        for (int i = 0; i < 4; i++) sample(1_750_000_000L + i * 250_000_000L, 49 + i * 14);
        verify(telemetry, never()).addData(eq("Measured RPM"), anyString(), anyDouble());
        sample(2_750_000_000L, 105);
        verify(telemetry).addData("Measured RPM", "%.1f", 120.0);
    }

    @Test public void motorRunsAtPointOneWithoutAnyCalibration() {
        op.start();
        clearInvocations(motor);
        MotorRpmTest.power = 0.1;
        op.loop();
        verify(motor).setPower(0.1);
        verify(motor, never()).getMotorType();
    }

    @Test public void calibrationMarksAndClearDoNotInterruptMotorPower() {
        op.start();
        MotorRpmTest.power = 0.1;
        clearInvocations(motor);
        MotorRpmTest.markStart = true; op.loop();
        when(data.getMotorCurrentPosition(0)).thenReturn(280);
        MotorRpmTest.markEnd = true; op.loop();
        MotorRpmTest.clearCalibration = true; op.loop();
        verify(motor, times(3)).setPower(0.1);
        verify(motor, never()).setPower(0);
        assertEquals(0.1, MotorRpmTest.power, 0);
    }

    @Test public void uncalibratedMotorShowsRawEncoderSpeedWithoutInventingRpm() {
        op.start();
        MotorRpmTest.power = 0.1;
        for (int i = 0; i <= 4; i++) sample(i * 250_000_000L, i * 7);
        verify(telemetry).addData("Encoder speed (counts/second)", "%.1f", 28.0);
        verify(telemetry, never()).addData(eq("Measured RPM"), anyString(), anyDouble());
    }

    @Test public void observedTurnsWhileMotorRunsEnableActualRpm() {
        op.start();
        MotorRpmTest.power = 0.1;
        calibrateAt(0, 280);
        for (int i = 0; i <= 4; i++) sample(i * 250_000_000L, i * 7);
        verify(telemetry).addData("Measured RPM", "%.1f", 60.0);
        assertEquals(0.1, MotorRpmTest.power, 0);
    }

    @Test public void fakeHubSampleStopsInsteadOfShowingAFabricatedRpm() {
        op.start();
        calibrateAt(0, 280);
        clearInvocations(motor);
        when(data.isFake()).thenReturn(true);
        MotorRpmTest.power = 0.6;
        assertThrows(IllegalStateException.class, op::loop);
        assertEquals(0, MotorRpmTest.power, 0);
        verify(motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clearInvocations(motor, controlHub);
        op.loop();
        verifyNoInteractions(motor, controlHub);
    }

    private void calibrateAt(int endPosition, int counts) {
        when(data.getMotorCurrentPosition(0)).thenReturn(endPosition - counts);
        MotorRpmTest.markStart = true; op.loop();
        when(data.getMotorCurrentPosition(0)).thenReturn(endPosition);
        MotorRpmTest.markEnd = true; op.loop();
    }

    private void sample(long timestamp, int position) {
        now.set(timestamp);
        when(data.getMotorCurrentPosition(0)).thenReturn(position);
        op.loop();
    }
}
