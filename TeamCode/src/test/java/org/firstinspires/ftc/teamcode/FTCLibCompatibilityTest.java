package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import org.junit.Test;
import static org.junit.Assert.assertEquals;

/** Compile/runtime characterization of the separately installed FTCLib controller API. */
public class FTCLibCompatibilityTest {
    @Test public void positionControllerRespondsToTargetChangesAndSettlesAtTarget() {
        PIDController controller = new PIDController(0.1, 0, 0);
        assertEquals(0.6, controller.calculate(4, 10), 1e-9);
        assertEquals(-0.4, controller.calculate(4, 0), 1e-9);
        assertEquals(0, controller.calculate(0, 0), 1e-9);
    }
}
