package org.firstinspires.ftc.teamcode.telemetry;

import com.bylazar.configurables.GenericTypeJson;
import com.bylazar.configurables.GlobalConfigurables;
import com.bylazar.configurables.Plugin;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.variables.BaseTypes;
import com.bylazar.configurables.variables.generics.GenericField;
import org.firstinspires.ftc.teamcode.opmodes.ConfigureColorRangefinder;
import org.firstinspires.ftc.teamcode.opmodes.testing.AxonTurretTest;
import org.firstinspires.ftc.teamcode.opmodes.testing.FieldMap;
import org.firstinspires.ftc.teamcode.opmodes.testing.HiveAutoAimTest;
import org.firstinspires.ftc.teamcode.opmodes.testing.ShooterTuning;
import org.firstinspires.ftc.teamcode.pedroPathing.MarrowConfig;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.junit.After;
import org.junit.Test;

import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.*;

/** Actual reload-aware Panels scanner and setters, including nested flywheel PID settings. */
public class PanelsConfigurablesTest {
    private final ClassLoader loader = new ClassLoader(getClass().getClassLoader()) { };

    @After public void unload() { Plugin.INSTANCE.beforeUnload(loader); }

    @Test public void allTuningGroupsAreDiscoverableAndInternalPoseTransferIsHidden() {
        Class<?>[] groups = {Launcher.class, AxonTurret.class, HiveAutoAim.class, HiveVisionConfig.class,
                HivePedroConfig.class, Limelight.class, Drivetrain.class, ExampleSubsystem.class,
                MarrowConfig.class, ConfigureColorRangefinder.class, ShooterTuning.class,
                AxonTurretTest.class, HiveAutoAimTest.class, FieldMap.class};
        for (Class<?> group : groups) {
            assertTrue(group.getSimpleName(), group.isAnnotationPresent(Configurable.class));
            Plugin.INSTANCE.scan(loader, group);
            List<GenericField> fields = GlobalConfigurables.INSTANCE.getJvmFields().get(loader).stream()
                    .filter(field -> field.getReference().getDeclaringClass() == group).collect(Collectors.toList());
            assertFalse(group.getSimpleName(), fields.isEmpty());
            for (GenericField field : fields) {
                BaseTypes type = field.getToJsonType().getType();
                assertNotEquals(group.getSimpleName() + "." + field.getName(), BaseTypes.JSON_ERROR, type);
                assertNotEquals(group.getSimpleName() + "." + field.getName(), BaseTypes.ERROR, type);
                assertNotEquals("poseTransfer", field.getName());
            }
        }
    }

    @Test public void panelsEditsTheSameNestedShooterGainUsedByTheController() {
        double before = Launcher.big.kP;
        try {
            Plugin.INSTANCE.scan(loader, Launcher.class);
            GenericTypeJson big = GlobalConfigurables.INSTANCE.getJvmFields().get(loader).stream()
                    .filter(field -> field.getName().equals("big")).findFirst().get().getToJsonType();
            GenericTypeJson gain = big.getCustomValues().stream()
                    .filter(field -> field.getFieldName().equals("kP")).findFirst().get();
            assertTrue(GlobalConfigurables.INSTANCE.getFieldsMap().get(gain.getId()).setValue("0.0042"));
            assertEquals(.0042, Launcher.big.kP, 0);
        } finally { Launcher.big.kP = before; }
    }

    @Test public void configurableFieldsAreReleasedWhenSlothUnloadsTheirClassloader() {
        Plugin.INSTANCE.scan(loader, HiveVisionConfig.class);
        List<String> ids = GlobalConfigurables.INSTANCE.getLoadedFieldsMap().get(loader);
        assertFalse(ids.isEmpty());
        Plugin.INSTANCE.beforeUnload(loader);
        assertFalse(GlobalConfigurables.INSTANCE.getJvmFields().containsKey(loader));
        for (String id : ids) assertFalse(GlobalConfigurables.INSTANCE.getFieldsMap().containsKey(id));
    }
}
