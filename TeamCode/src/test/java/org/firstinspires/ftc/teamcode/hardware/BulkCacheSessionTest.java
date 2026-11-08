package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import org.junit.Test;

import java.util.Arrays;

import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class BulkCacheSessionTest {
    private final LynxModule controlHub = mock(LynxModule.class), expansionHub = mock(LynxModule.class);
    private final BulkCacheSession cache = new BulkCacheSession();

    @Test public void eachLoopClearsBothHubsAndStopRestoresTheirOwnModesOnce() {
        when(controlHub.getBulkCachingMode()).thenReturn(AUTO);
        when(expansionHub.getBulkCachingMode()).thenReturn(OFF);
        cache.enable(Arrays.asList(controlHub, expansionHub));
        verify(controlHub).setBulkCachingMode(MANUAL);
        verify(expansionHub).setBulkCachingMode(MANUAL);
        cache.clear();
        cache.clear();
        verify(controlHub, times(2)).clearBulkCache();
        verify(expansionHub, times(2)).clearBulkCache();
        cache.close();
        cache.close();
        verify(controlHub).setBulkCachingMode(AUTO);
        verify(expansionHub).setBulkCachingMode(OFF);
        clearInvocations(controlHub, expansionHub);
        cache.clear();
        verifyNoInteractions(controlHub, expansionHub);
    }

    @Test public void partiallyFailedInitRestoresEveryModeAlreadyCaptured() {
        when(controlHub.getBulkCachingMode()).thenReturn(AUTO);
        when(expansionHub.getBulkCachingMode()).thenReturn(OFF);
        RuntimeException failure = new IllegalStateException("Hub mode write failed");
        doThrow(failure).when(expansionHub).setBulkCachingMode(MANUAL);
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> cache.enable(Arrays.asList(controlHub, expansionHub))));
        verify(controlHub).setBulkCachingMode(AUTO);
        verify(expansionHub).setBulkCachingMode(OFF);
        clearInvocations(controlHub, expansionHub);
        cache.close();
        verifyNoInteractions(controlHub, expansionHub);
    }

    @Test public void oneDisconnectedHubCannotPreventRestoringTheOther() {
        when(controlHub.getBulkCachingMode()).thenReturn(AUTO);
        when(expansionHub.getBulkCachingMode()).thenReturn(OFF);
        cache.enable(Arrays.asList(controlHub, expansionHub));
        RuntimeException failure = new IllegalStateException("Control hub disconnected");
        doThrow(failure).when(controlHub).setBulkCachingMode(AUTO);
        assertSame(failure, assertThrows(RuntimeException.class, cache::close));
        verify(expansionHub).setBulkCachingMode(OFF);
    }
}
