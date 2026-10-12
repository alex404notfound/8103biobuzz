package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HiveVisionConfig;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.*;

public class HiveVisionOpModeTest {
    private final Limelight3A camera = mock(Limelight3A.class);
    private final HardwareMap hardware = mock(HardwareMap.class);
    private HiveVisionTest op;

    @Before public void setUp() {
        op = new HiveVisionTest() {
            @Override protected Telemetry withDashboard(Telemetry driverTelemetry) { return driverTelemetry; }
        };
        op.hardwareMap = hardware;
        op.telemetry = mock(Telemetry.class);
        op.gamepad1 = mock(Gamepad.class);
        when(hardware.get(Limelight3A.class, HiveVisionConfig.hardwareName)).thenReturn(camera);
        when(camera.pipelineSwitch(HiveVisionConfig.pipelineIndex)).thenReturn(true);
    }

    @Test public void initRequiresOnlyCameraAndSamplesOneSnapshotPerInitOrActiveLoop() {
        op.init();
        verify(hardware).get(Limelight3A.class, HiveVisionConfig.hardwareName);
        verifyNoMoreInteractions(hardware);
        verify(camera).pipelineSwitch(HiveVisionConfig.pipelineIndex);
        verify(camera).start();
        op.init_loop();
        op.start();
        op.loop();
        verify(camera, times(2)).getLatestResult();
    }

    @Test public void failedPipelineRequestStopsCameraAndDoesNotStartPolling() {
        when(camera.pipelineSwitch(HiveVisionConfig.pipelineIndex)).thenReturn(false);
        assertThrows(IllegalStateException.class, op::init);
        verify(camera).stop();
        verify(camera, never()).start();
        op.init_loop();
        op.loop();
        verify(camera, never()).getLatestResult();
    }

    @Test public void stopDuringInitIsIdempotentAndCannotResumePolling() {
        op.init();
        op.stop();
        clearInvocations(camera);
        op.start();
        op.init_loop();
        op.loop();
        op.stop();
        verifyNoInteractions(camera);
    }

    @Test public void pollingFailureStopsCamera() {
        op.init();
        when(camera.getLatestResult()).thenThrow(new IllegalStateException("disconnected"));
        assertThrows(IllegalStateException.class, op::init_loop);
        verify(camera).stop();
    }

    @Test public void wrongPipelineIsReportedWithoutSwitchingAgainEveryLoop() {
        LLResult frame = mock(LLResult.class);
        when(frame.getPipelineIndex()).thenReturn(0);
        when(camera.getLatestResult()).thenReturn(frame);
        op.init();
        op.init_loop();
        op.loop();
        verify(camera, times(1)).pipelineSwitch(HiveVisionConfig.pipelineIndex);
        verify(op.telemetry, times(2)).addData("Vision status", "Waiting for detector pipeline " + HiveVisionConfig.pipelineIndex);
    }
}
