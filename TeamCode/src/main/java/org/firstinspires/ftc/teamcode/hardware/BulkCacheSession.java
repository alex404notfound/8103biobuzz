package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Per-OpMode ownership of manual bulk caching, including restoration after partial INIT. */
public final class BulkCacheSession implements AutoCloseable {
    private final Map<LynxModule, LynxModule.BulkCachingMode> previousModes = new LinkedHashMap<>();

    public void enable(HardwareMap hardwareMap) { enable(hardwareMap.getAll(LynxModule.class)); }

    public void enable(List<LynxModule> hubs) {
        close();
        try {
            for (LynxModule hub : hubs) {
                previousModes.put(hub, hub.getBulkCachingMode());
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } catch (RuntimeException failure) {
            try { close(); }
            catch (RuntimeException restoreFailure) {
                if (failure != restoreFailure) failure.addSuppressed(restoreFailure);
            }
            throw failure;
        }
    }

    public void clear() {
        for (LynxModule hub : previousModes.keySet()) hub.clearBulkCache();
    }

    @Override public void close() {
        RuntimeException failure = null;
        for (Map.Entry<LynxModule, LynxModule.BulkCachingMode> entry : previousModes.entrySet()) {
            try { entry.getKey().setBulkCachingMode(entry.getValue()); }
            catch (RuntimeException fault) {
                if (failure == null) failure = fault;
                else if (failure != fault) failure.addSuppressed(fault);
            }
        }
        previousModes.clear();
        if (failure != null) throw failure;
    }
}
