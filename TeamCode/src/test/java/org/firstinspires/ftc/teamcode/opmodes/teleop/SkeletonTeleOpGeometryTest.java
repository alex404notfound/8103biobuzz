package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Field;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.junit.After;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class SkeletonTeleOpGeometryTest {
    private final Alliance previousAlliance = Alliance.current;
    private final boolean previousConfigured = FieldConstants.START_POSE_CONFIGURED;

    @After public void restoreConfiguration() {
        Alliance.current = previousAlliance;
        FieldConstants.START_POSE_CONFIGURED = previousConfigured;
    }

    @Test public void headingLockRotatesForBlue() throws Exception {
        Harness opMode = new Harness();
        when(opMode.gamepad1.squareWasPressed()).thenReturn(true);
        Alliance.current = Alliance.RED;
        opMode.onLoop();
        verify(opMode.drive).lockHeading(90);
        Alliance.current = Alliance.BLUE;
        opMode.onLoop();
        verify(opMode.drive).lockHeading(270);
    }

    @Test public void operatorCannotInjectUnmeasuredStartPose() throws Exception {
        Harness opMode = new Harness();
        FieldConstants.START_POSE_CONFIGURED = false;
        when(opMode.gamepad2.crossWasPressed()).thenReturn(true);
        opMode.onLoop();
        verify(opMode.drive, never()).setPose(any());
        verify(opMode.telemetryOutput).addLine(contains("Corner reset disabled"));
    }

    @Test public void configuredBlueResetUsesRotatedPositionAndHeading() throws Exception {
        Harness opMode = new Harness();
        FieldConstants.START_POSE_CONFIGURED = true;
        Alliance.current = Alliance.BLUE;
        when(opMode.gamepad2.crossWasPressed()).thenReturn(true);
        opMode.onLoop();
        ArgumentCaptor<Pose> pose = ArgumentCaptor.forClass(Pose.class);
        verify(opMode.drive).setPose(pose.capture());
        assertEquals(144 - FieldConstants.RED_CORNER_START.x(), pose.getValue().x(), 1e-9);
        assertEquals(144 - FieldConstants.RED_CORNER_START.y(), pose.getValue().y(), 1e-9);
        assertEquals(3 * Math.PI / 2, pose.getValue().heading(), 1e-9);
    }

    private static class Harness extends SkeletonTeleOp {
        final Drivetrain drive = mock(Drivetrain.class);
        final Telemetry telemetryOutput = mock(Telemetry.class);
        Harness() throws Exception {
            robot = mock(Robot.class);
            setRobotField("drivetrain", drive);
            setRobotField("telemetry", telemetryOutput);
            gamepad1 = mock(Gamepad.class);
            gamepad2 = mock(Gamepad.class);
        }
        private void setRobotField(String name, Object value) throws Exception {
            Field field = Robot.class.getField(name);
            field.setAccessible(true);
            field.set(robot, value);
        }
    }
}
