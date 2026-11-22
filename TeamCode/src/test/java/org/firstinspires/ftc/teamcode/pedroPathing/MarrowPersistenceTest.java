package org.firstinspires.ftc.teamcode.pedroPathing;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.skeletonarmy.marrow.internal.FileHandler;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import static org.junit.Assert.assertEquals;

/** Checks Marrow's actual JSON configuration against the upgraded Jackson runtime. */
public class MarrowPersistenceTest {
    private ObjectMapper mapper() throws ReflectiveOperationException {
        // Avoid Android external storage while exercising the mapper used by FileHandler.
        Field field = FileHandler.class.getDeclaredField("MAPPER");
        field.setAccessible(true);
        return (ObjectMapper) field.get(null);
    }

    @Test public void readsExistingTypedSettings() throws Exception {
        String legacy = "[\"java.util.HashMap\",{\"rpm\":3000.0,"
                + "\"positions\":[\"java.util.ArrayList\",[1.0,2.0]]}]";
        Map<String, Object> loaded = mapper().readValue(legacy,
                new TypeReference<HashMap<String, Object>>() { });
        assertEquals(3000.0, loaded.get("rpm"));
        assertEquals(Arrays.asList(1.0, 2.0), loaded.get("positions"));
    }

    @Test public void roundTripsTypedSettingsWithNestedCollections() throws Exception {
        Map<String, Object> settings = new HashMap<>();
        settings.put("rpm", 3000.0);
        settings.put("enabled", true);
        settings.put("positions", new ArrayList<>(Arrays.asList(1.0, 2.0)));
        Map<String, Object> nested = new HashMap<>();
        nested.put("offset", 1.5);
        settings.put("calibration", nested);
        ObjectMapper mapper = mapper();
        Map<String, Object> loaded = mapper.readValue(mapper.writeValueAsString(settings),
                new TypeReference<HashMap<String, Object>>() { });
        assertEquals(settings, loaded);
    }
}
