package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.field.Canvas;
import com.bylazar.field.Circle;
import com.bylazar.field.Line;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.ShotTable;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.robot.ShotTableStore;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Launcher.Ball;
import org.firstinspires.ftc.teamcode.testutil.MemoryShotStorage;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import java.util.Collections;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;
import static org.firstinspires.ftc.teamcode.opmodes.testing.ShooterTuning.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class ShooterTuningTest {
    private final Drivetrain drive = mock(Drivetrain.class);
    private final Launcher launcher = mock(Launcher.class);
    private final Telemetry telemetry = mock(Telemetry.class);
    private final MemoryShotStorage disk = new MemoryShotStorage();
    private final ShotTableStore store = new ShotTableStore(disk);
    private final AtomicLong now = new AtomicLong();
    private final Gamepad one = new Gamepad(), two = new Gamepad();
    private ShooterTuning op;
    private Pose pose = new Pose(72, 24, 0);
    private Velocity velocity = new Velocity(0, 0, 0);
    private Ball activeBall = Ball.SMALL_POLLEN;
    private double target;
    private boolean ready = true, moving;

    @Before public void setUp() {
        defaults();
        when(drive.isLocalizationReady()).thenReturn(true);
        when(drive.getPose()).thenAnswer(call -> pose);
        when(drive.getVelocity()).thenAnswer(call -> velocity);
        doAnswer(call -> { pose = call.getArgument(0); return null; }).when(drive).setStartingPose(any());
        doAnswer(call -> { moving = true; return null; }).when(drive).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
        doAnswer(call -> { moving = false; return null; }).when(drive).stop();
        doAnswer(call -> { activeBall = call.getArgument(0); return null; }).when(launcher).selectBall(any());
        when(launcher.getManualRpm()).thenAnswer(call -> activeBall == Ball.LARGE_NECTAR ? Launcher.largeBallRpm : Launcher.smallBallRpm);
        when(launcher.getTargetRpm()).thenAnswer(call -> target);
        when(launcher.isAtSpeed()).thenAnswer(call -> ready && target > 0);
        doAnswer(call -> { target = call.getArgument(0); return null; }).when(launcher).requestRpm(anyDouble());
        doAnswer(call -> { target = 0; return null; }).when(launcher).idle();
        doAnswer(call -> { target = 0; return null; }).when(launcher).stop();
        op = new ShooterTuning(drive, launcher, store, now::get, () -> mock(Telemetry.class));
        op.hardwareMap = mock(HardwareMap.class);
        when(op.hardwareMap.getAll(LynxModule.class)).thenReturn(Collections.emptyList());
        op.telemetry = telemetry;
        op.gamepad1 = new Gamepad(); op.gamepad2 = new Gamepad();
    }

    @After public void tearDown() { op.stop(); defaults(); }
    private static void defaults() {
        selectedBall = Ball.SMALL_POLLEN; selectedCell = HiveCell.RED_AUDIENCE;
        automaticLookup = false; rpmStep = 50; driveScale = .35;
        startX = 72; startY = 24; startHeadingDegrees = 90;
        Launcher.smallBallRpm = 2000; Launcher.largeBallRpm = 3000;
    }

    @Test public void initAndStartRequireFreshReleasedControlsBeforeMotorCanRun() {
        one.right_trigger = 1;
        initializeAt(40); op.loop();
        verify(launcher, never()).requestRpm(anyDouble());
        op.start(); tick();
        assertEquals(0, target, 0);
        one.right_trigger = 0; tick();
        one.right_trigger = 1; tick();
        assertEquals(2000, target, 0);
        op.stop(); clearInvocations(launcher, drive);
        op.start(); tick(); op.init_loop();
        verifyNoInteractions(launcher, drive);
    }

    @Test public void saveCapturesSettledPoseRpmAndBallThenPersistsAcrossNewStore() throws Exception {
        startAt(40); spin();
        disk.beforeWrite = () -> { assertEquals(0, target, 0); assertFalse(moving); };
        one.a = true; tick(); tick(); tick();
        assertEquals(1, disk.writes); // Holding A never writes twice.
        assertEquals(0, target, 0); // Saving disarms until release.
        Map<Ball, ShotTable> recovered = new ShotTableStore(disk).load();
        ShotTable.Point sample = recovered.get(Ball.SMALL_POLLEN).points().get(0);
        assertEquals(40, sample.distance, 1e-9);
        assertEquals(2000, sample.rpm, 0);
        assertEquals(pose.x(), sample.x, 0);
        assertEquals(selectedCell, sample.cell);
        assertTrue(recovered.get(Ball.LARGE_NECTAR).points().isEmpty());
    }

    @Test public void saveRefusesMovingUnsettledOrUnreferencedRobot() {
        startAt(40);
        velocity = new Velocity(5, 0, 0); spin(); pressSave();
        assertEquals(0, disk.writes);
        velocity = new Velocity(0, 0, 0); tick(); pressSave();
        assertEquals(0, disk.writes);
        now.addAndGet(400_000_000L); ready = false; pressSave();
        assertEquals(0, disk.writes);
        ready = true; two.left_bumper = true; pressSave();
        assertEquals(0, disk.writes);
        two.left_bumper = false; pressSave();
        assertEquals(1, disk.writes);
    }

    @Test public void missingMeasuredPoseAllowsManualTuningButNeverSavingOrLookup() {
        op.init(); op.start(); tick(); spin(); pressSave();
        assertEquals(2000, target, 0);
        assertEquals(0, disk.writes);
        one.right_trigger = 0; automaticLookup = true; tick(); spin();
        assertEquals(0, target, 0);
    }

    @Test public void invalidSampleAtCellCenterIsRefusedWithoutEndingOpMode() {
        startAt(.5); spin(); pressSave();
        assertEquals(0, disk.writes);
        verify(launcher, never()).stop();
        assertEquals(2000, target, 0);
    }

    @Test public void lookupInterpolatesWhileMovingWithoutWritingAndStopsOutsideRange() throws Exception {
        preload();
        startAt(50); useLookup(); spin();
        assertEquals(2300, target, 1e-9);
        setRange(45); velocity = new Velocity(3, 0, 0); tick();
        assertEquals(2150, target, 1e-9);
        setRange(65); tick();
        assertEquals(0, target, 0);
        setRange(50); tick();
        assertEquals(2300, target, 1e-9);
        assertEquals(1, disk.writes); // Only preload wrote to storage.
        pressSave(); assertEquals(1, disk.writes); // No self-calibration in lookup mode.
    }

    @Test public void ballOrCellSwitchWhileHeldRequiresReleaseAndUsesSeparateTable() throws Exception {
        preload(); startAt(50); useLookup(); spin();
        assertEquals(2300, target, 1e-9);
        one.dpad_right = true; tick();
        assertEquals(0, target, 0);
        one.dpad_right = false; one.right_trigger = 0; tick(); spin();
        assertEquals(3500, target, 1e-9);
        selectedCell = HiveCell.BLUE_AUDIENCE; tick();
        assertEquals(0, target, 0);
        one.right_trigger = 0; tick(); setRange(50); spin();
        assertEquals(3500, target, 1e-9);
    }

    @Test public void lookupNeedsTwoDistancesForTheSelectedBall() throws Exception {
        Map<Ball, ShotTable> tables = ShotTableStore.empty();
        tables.put(Ball.SMALL_POLLEN, new ShotTable().with(point(40, 2000)));
        store.save(tables);
        startAt(40); useLookup(); spin();
        assertEquals(0, target, 0);
    }

    @Test public void savingBothBallsNeverOverwritesTheOtherTable() throws Exception {
        startAt(40); spin(); pressSave();
        one.right_trigger = 0; selectedBall = Ball.LARGE_NECTAR; tick();
        now.addAndGet(400_000_000L); spin(); pressSave();
        Map<Ball, ShotTable> recovered = store.load();
        assertEquals(2000, recovered.get(Ball.SMALL_POLLEN).points().get(0).rpm, 0);
        assertEquals(3000, recovered.get(Ball.LARGE_NECTAR).points().get(0).rpm, 0);
    }

    @Test public void failedSavePreservesTheCurrentLookupAndPersistentFile() throws Exception {
        preload(); String prior = disk.content;
        startAt(50); Launcher.smallBallRpm = 2400; spin();
        disk.failWrite = true; pressSave();
        assertEquals(0, target, 0);
        assertEquals(prior, disk.content);
        useLookup(); spin();
        assertEquals(2300, target, 1e-9);
    }

    @Test public void undoRestoresPreviousSavedTableDurably() throws Exception {
        preload(); startAt(50); Launcher.smallBallRpm = 2400; spin(); pressSave();
        assertEquals(2400, store.load().get(Ball.SMALL_POLLEN).lookup(50).rpm, 1e-9);
        one.right_trigger = 0; tick(); one.b = true; tick();
        assertEquals(2300, store.load().get(Ball.SMALL_POLLEN).lookup(50).rpm, 1e-9);
        assertEquals(2, store.load().get(Ball.SMALL_POLLEN).points().size());
    }

    @Test public void corruptFileIsPreservedAndDisablesSaveAndLookup() {
        disk.content = "{bad data";
        startAt(40); spin(); pressSave();
        assertEquals(0, disk.writes); assertEquals("{bad data", disk.content);
        useLookup(); spin(); assertEquals(0, target, 0);
    }

    @Test public void localizationLossStopsLookupAndRequiresNewInitReference() throws Exception {
        preload(); startAt(50); useLookup(); spin();
        when(drive.isLocalizationReady()).thenReturn(false); tick();
        assertEquals(0, target, 0);
        when(drive.isLocalizationReady()).thenReturn(true);
        one.right_trigger = 0; tick(); two.a = true; tick(); spin();
        assertEquals(0, target, 0);
        verify(drive, times(1)).setStartingPose(any()); // Active A cannot silently relocate robot.
    }

    @Test public void cancelAndModeChangeDisarmHeldShooterAndDrive() {
        startAt(40); spin(); two.left_bumper = true; two.left_stick_y = -1; tick();
        verify(drive).driveRobotRelative(eq(.35), doubleThat(v -> v == 0), doubleThat(v -> v == 0));
        assertTrue(moving);
        one.x = true; tick(); assertFalse(moving); assertEquals(0, target, 0);
        one.x = false; tick(); assertEquals(0, target, 0);
        one.right_trigger = 0; two.left_bumper = false; tick(); spin();
        assertEquals(2000, target, 0);
        one.y = true; tick(); assertEquals(0, target, 0);
    }

    @Test public void directionsAndManualRpmButtonsRouteOnlyTheSelectedBall() {
        startAt(40);
        one.left_bumper = true; tick(); verify(launcher).requestMotorTest();
        one.right_trigger = 1; tick();
        verify(launcher, never()).requestRpm(anyDouble()); // Conflicting holds stop.
        one.right_trigger = 0; one.left_bumper = false; one.dpad_up = true; tick(); tick();
        verify(launcher, times(1)).adjustRpm(50);
        one.dpad_up = false; one.dpad_down = true; tick();
        verify(launcher).adjustRpm(-50);
    }

    @Test public void combinedTranslationAndTurnStayWithinTuningPowerLimit() {
        startAt(40);
        two.left_bumper = true;
        two.left_stick_y = two.left_stick_x = two.right_stick_x = -1;
        tick();
        verify(drive).driveRobotRelative(.35 / 3, .35 / 3, .35 / 3);
        driveScale = .5; tick();
        verify(drive).driveRobotRelative(.5 / 3, .5 / 3, .5 / 3);
        driveScale = .6; clearInvocations(drive); tick();
        assertFalse(moving);
        verify(drive, never()).driveRobotRelative(anyDouble(), anyDouble(), anyDouble());
    }

    @Test public void driveFailureStopsShooterAndClosesOpMode() {
        startAt(40); spin();
        doThrow(new IllegalStateException("Pinpoint disconnected")).when(drive).periodic();
        assertThrows(IllegalStateException.class, this::tick);
        assertEquals(0, target, 0); verify(launcher).stop();
        clearInvocations(launcher); tick(); verifyNoInteractions(launcher);
    }

    @Test public void stopStillStopsShooterIfDriveStopThrows() {
        startAt(40); spin();
        doThrow(new IllegalStateException("Drive disconnected")).when(drive).stop();
        assertThrows(IllegalStateException.class, op::stop);
        assertEquals(0, target, 0); verify(launcher).stop();
    }

    @Test public void fieldOverlayIncludesSelectedTargetSavedPointAndRobotDistanceLine() throws Exception {
        preload(); startAt(50);
        ArgumentCaptor<Consumer<Canvas>> overlay = ArgumentCaptor.forClass(Consumer.class);
        verify(drive).setFieldOverlay(overlay.capture());
        Canvas canvas = new Canvas();
        overlay.getValue().accept(canvas);
        double x = ShotTable.targetX(selectedCell), y = ShotTable.targetY(selectedCell);
        Style target = new Style("transparent", "#fbbf24", .2);
        Style saved = new Style("#fbbf24", "transparent", 0);
        assertTrue(canvas.getItems().contains(new Circle(x, y, 4, target)));
        assertTrue(canvas.getItems().contains(new Line(pose.x(), pose.y(), x, y, target)));
        assertTrue(canvas.getItems().contains(new Circle(x + 40, y, 1.5, saved)));
        assertTrue(canvas.getItems().contains(new Circle(x + 60, y, 1.5, saved)));
    }

    private void initializeAt(double distance) {
        startX = ShotTable.targetX(selectedCell) + distance; startY = ShotTable.targetY(selectedCell);
        startHeadingDegrees = 180;
        op.init(); two.a = true; initTick(); two.a = false; initTick();
    }
    private void startAt(double distance) {
        initializeAt(distance); op.start(); tick();
        now.addAndGet(400_000_000L); tick();
    }
    private void initTick() { copyInputs(); now.addAndGet(20_000_000L); op.init_loop(); }
    private void tick() { copyInputs(); now.addAndGet(20_000_000L); op.loop(); }
    private void copyInputs() { op.gamepad1.copy(one); op.gamepad2.copy(two); }
    private void spin() { one.right_trigger = 1; tick(); }
    private void pressSave() { one.a = true; tick(); one.a = false; tick(); }
    private void useLookup() { one.right_trigger = 0; automaticLookup = true; tick(); }
    private void setRange(double distance) {
        pose = new Pose(ShotTable.targetX(selectedCell) + distance, ShotTable.targetY(selectedCell), Math.PI);
    }
    private ShotTable.Point point(double distance, double rpm) {
        HiveCell cell = HiveCell.RED_AUDIENCE;
        return new ShotTable.Point(new Pose(ShotTable.targetX(cell) + distance, ShotTable.targetY(cell), Math.PI), cell, rpm);
    }
    private void preload() throws Exception {
        Map<Ball, ShotTable> tables = ShotTableStore.empty();
        tables.put(Ball.SMALL_POLLEN, new ShotTable().with(point(40, 2000)).with(point(60, 2600)));
        tables.put(Ball.LARGE_NECTAR, new ShotTable().with(point(40, 3000)).with(point(60, 4000)));
        store.save(tables);
    }
}
