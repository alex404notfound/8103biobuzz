package org.firstinspires.ftc.teamcode.robot;

import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.RobotState;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextFTCOpModeScannerKt;
import dev.nextftc.robot.opmode.OpModeMetaCheckResult;
import dev.nextftc.robot.opmode.OpModeConstructorCheckResult;
import kotlin.jvm.JvmClassMappingKt;
import org.firstinspires.ftc.teamcode.opmodes.autos.BlueSkeletonAuto;
import org.firstinspires.ftc.teamcode.opmodes.autos.RedSkeletonAuto;
import org.firstinspires.ftc.teamcode.opmodes.teleop.SkeletonTeleOp;
import org.firstinspires.ftc.teamcode.opmodes.teleop.SkeletonVisionTeleOp;
import org.firstinspires.ftc.teamcode.opmodes.testing.LimelightCheck;
import org.firstinspires.ftc.teamcode.opmodes.testing.HivePedroTest;
import org.firstinspires.ftc.teamcode.testutil.NextOpModeHarness;
import org.junit.Test;

import java.lang.reflect.Field;

import static org.junit.Assert.*;

public class NextFtcRegistrationTest {
    @Test public void robotDiscoveryDoesNotTouchHardwareAndEveryMigratedOpModeHasAUsableRegistration() throws Exception {
        Robot robot = new Robot();
        assertTrue(robot instanceof NextRobot);
        assertTrue(robot.getMechanisms().isEmpty());
        assertNull(robot.drivetrain);
        Field robotClass = RobotState.class.getDeclaredField("robotClass");
        Field robotInstance = RobotState.class.getDeclaredField("robotInstance");
        robotClass.setAccessible(true);
        robotInstance.setAccessible(true);
        Object oldClass = robotClass.get(null), oldInstance = robotInstance.get(null);
        try {
            robotClass.set(null, JvmClassMappingKt.getKotlinClass(Robot.class));
            robotInstance.set(null, robot);
            check(RedSkeletonAuto.class, "Red Skeleton", robot);
            check(BlueSkeletonAuto.class, "Blue Skeleton", robot);
            check(SkeletonTeleOp.class, "Skeleton TeleOp", robot);
            check(SkeletonVisionTeleOp.class, "Skeleton Vision TeleOp", robot);
            check(LimelightCheck.class, "Limelight Check", robot);
            check(HivePedroTest.class, "Hive Pedro Test", robot);
        } finally {
            robotClass.set(null, oldClass);
            robotInstance.set(null, oldInstance);
        }
    }

    private <T extends RobotOpMode> void check(Class<T> type, String name, Robot robot) {
        OpModeMetaCheckResult meta = NextFTCOpModeScannerKt.opModeMetaFromClass(JvmClassMappingKt.getKotlinClass(type));
        assertTrue(meta instanceof OpModeMetaCheckResult.FoundAnnotation);
        assertEquals(name, ((OpModeMetaCheckResult.FoundAnnotation) meta).getMeta().name);
        OpModeConstructorCheckResult constructor = NextFTCOpModeScannerKt.opModeConstructorFromClass(
                JvmClassMappingKt.getKotlinClass(type));
        assertTrue(constructor instanceof OpModeConstructorCheckResult.FoundConstructor);
        NextOpMode instance = NextOpModeHarness.construct(() ->
                ((OpModeConstructorCheckResult.FoundConstructor) constructor).getConstructor().invoke());
        assertEquals(type, instance.getClass());
        assertSame(robot, ((RobotOpMode) instance).robot);
    }
}
