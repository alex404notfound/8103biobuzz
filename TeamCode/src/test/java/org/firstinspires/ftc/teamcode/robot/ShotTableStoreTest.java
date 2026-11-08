package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.math.ShotTable;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.subsystems.Launcher.Ball;
import org.firstinspires.ftc.teamcode.testutil.MemoryShotStorage;
import org.json.JSONObject;
import org.junit.Test;
import java.io.IOException;
import java.util.Map;
import static org.junit.Assert.*;

public class ShotTableStoreTest {
    private final MemoryShotStorage disk = new MemoryShotStorage();
    private final ShotTableStore store = new ShotTableStore(disk);

    @Test public void missingFileStartsWithBothEmptyTablesWithoutWriting() throws Exception {
        assertTrue(store.load().get(Ball.SMALL_POLLEN).points().isEmpty());
        assertTrue(store.load().get(Ball.LARGE_NECTAR).points().isEmpty());
        assertEquals(0, disk.writes);
    }
    @Test public void bothBallTypesAndFullPoseSurviveNewStoreInstance() throws Exception {
        Map<Ball, ShotTable> original = examples();
        store.save(original);
        Map<Ball, ShotTable> loaded = new ShotTableStore(disk).load();
        for (Ball ball : Ball.values()) {
            ShotTable.Point a = original.get(ball).points().get(0), b = loaded.get(ball).points().get(0);
            assertEquals(a.distance, b.distance, 1e-9);
            assertEquals(a.rpm, b.rpm, 0);
            assertEquals(a.x, b.x, 0); assertEquals(a.y, b.y, 0);
            assertEquals(a.heading, b.heading, 0); assertEquals(a.cell, b.cell);
        }
        assertNotEquals(loaded.get(Ball.SMALL_POLLEN).points().get(0).rpm,
                loaded.get(Ball.LARGE_NECTAR).points().get(0).rpm, 0);
    }
    @Test public void malformedMissingTableUnknownVersionOrChangedMapAreNeverSilentlyReset() throws Exception {
        store.save(examples()); String valid = disk.content;
        for (String invalid : new String[]{"", "{bad", "{}", valid.replace("\"version\": 1", "\"version\": 2"),
                valid.replace("SMALL_POLLEN", "UNKNOWN_BALL"), valid.replace("ROBOT_CENTER", "MUZZLE"),
                valid.replace("\"redX\": 59.25", "\"redX\": 60.0")}) {
            disk.content = invalid;
            assertThrows(IOException.class, store::load);
            assertEquals(invalid, disk.content);
        }
        assertEquals(1, disk.writes);
    }
    @Test public void distanceMismatchInvalidRpmAndDuplicateRowsAreRejected() throws Exception {
        store.save(examples()); String valid = disk.content;
        for (String key : new String[]{"distanceInches", "rpm", "x", "cell"}) {
            JSONObject root = new JSONObject(valid);
            JSONObject row = root.getJSONObject("tables").getJSONArray("SMALL_POLLEN").getJSONObject(0);
            row.put(key, key.equals("cell") ? "UNKNOWN" : -1);
            disk.content = root.toString();
            assertThrows(IOException.class, store::load);
        }
        JSONObject root = new JSONObject(valid);
        root.getJSONObject("tables").getJSONArray("SMALL_POLLEN")
                .put(root.getJSONObject("tables").getJSONArray("SMALL_POLLEN").getJSONObject(0));
        disk.content = root.toString();
        assertThrows(IOException.class, store::load);
    }
    @Test public void readAndWriteFailuresPropagateWithoutErasingSavedBytes() throws Exception {
        store.save(examples()); String valid = disk.content;
        disk.failRead = true; assertThrows(IOException.class, store::load);
        disk.failRead = false; disk.failWrite = true;
        assertThrows(IOException.class, () -> store.save(ShotTableStore.empty()));
        assertEquals(valid, disk.content);
        assertEquals(1, store.load().get(Ball.SMALL_POLLEN).points().size());
    }
    @Test public void missingTableCannotReplaceFileAndOversizedInputIsRejected() throws Exception {
        store.save(examples()); String valid = disk.content;
        Map<Ball, ShotTable> incomplete = examples(); incomplete.remove(Ball.LARGE_NECTAR);
        assertThrows(IOException.class, () -> store.save(incomplete));
        assertEquals(valid, disk.content);
        disk.content = new String(new char[100001]);
        assertThrows(IOException.class, store::load);
    }
    private Map<Ball, ShotTable> examples() {
        Map<Ball, ShotTable> tables = ShotTableStore.empty();
        tables.put(Ball.SMALL_POLLEN, new ShotTable().with(new ShotTable.Point(new Pose(20, 30, .7), HiveCell.RED_AUDIENCE, 2000)));
        tables.put(Ball.LARGE_NECTAR, new ShotTable().with(new ShotTable.Point(new Pose(120, 110, 1.8), HiveCell.BLUE_FAR, 3000)));
        return tables;
    }
}
