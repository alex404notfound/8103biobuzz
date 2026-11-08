package org.firstinspires.ftc.teamcode.testutil;

import org.firstinspires.ftc.teamcode.robot.ShotTableStore;
import java.io.IOException;

/** Persistent bytes shared across store instances, with controllable disk failures. */
public class MemoryShotStorage implements ShotTableStore.Storage {
    public String content;
    public boolean failRead, failWrite;
    public int writes;
    public Runnable beforeWrite = () -> {};
    @Override public String read() throws IOException {
        if (failRead) throw new IOException("Read failed");
        return content;
    }
    @Override public void write(String value) throws IOException {
        beforeWrite.run();
        writes++;
        if (failWrite) throw new IOException("Disk full");
        content = value;
    }
}
