package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilderFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static org.junit.Assert.*;

/** Checks the actual bundled configuration against the robot's hardware lookups. */
public class RobotConfigurationTest {
    private static final Path XML_DIRECTORY = Paths.get("src/main/res/xml");

    @Test public void oneConfigurationProvidesEveryMotorAndBothServosOnTheServoHub() throws Exception {
        try (Stream<Path> files = Files.list(XML_DIRECTORY)) {
            assertEquals(Arrays.asList("teamconfig8103.xml"), files
                    .map(path -> path.getFileName().toString())
                    .filter(name -> name.startsWith("teamconfig") && name.endsWith(".xml"))
                    .sorted().collect(Collectors.toList()));
        }

        Document document = DocumentBuilderFactory.newInstance().newDocumentBuilder()
                .parse(XML_DIRECTORY.resolve("teamconfig8103.xml").toFile());
        NodeList portals = document.getElementsByTagName("LynxUsbDevice");
        assertEquals(1, portals.getLength());
        Element portal = (Element) portals.item(0);
        assertEquals("(embedded)", portal.getAttribute("serialNumber"));

        Set<String> moduleAddresses = new HashSet<>();
        Map<String, Element> motors = new HashMap<>();
        Map<String, Element> servos = new HashMap<>();
        List<Element> modules = children(portal);
        assertEquals(3, modules.size());
        int lynxCount = 0, servoHubCount = 0;
        for (Element module : modules) {
            assertTrue("Duplicate hub address", moduleAddresses.add(module.getAttribute("port")));
            Set<String> ports = new HashSet<>();
            boolean servoHub = "ServoHub".equals(module.getTagName());
            if (servoHub) {
                servoHubCount++;
                assertEquals("3", module.getAttribute("port"));
            } else {
                lynxCount++;
                assertEquals("LynxModule", module.getTagName());
            }
            for (Element device : children(module)) {
                assertTrue("Duplicate device port", ports.add(device.getAttribute("port")));
                if (servoHub) {
                    assertEquals("Servo", device.getTagName());
                    assertNull(servos.put(device.getAttribute("name"), device));
                } else {
                    assertTrue(device.getTagName().endsWith("Motor"));
                    assertNull(motors.put(device.getAttribute("name"), device));
                }
            }
        }
        assertEquals(2, lynxCount);
        assertEquals(1, servoHubCount);
        assertEquals(new HashSet<>(Arrays.asList("173", "1", "3")), moduleAddresses);
        assertEquals(7, motors.size());
        assertEquals(new HashSet<>(Arrays.asList(Constants.drivetrainConfig.frontLeftName.get(),
                Constants.drivetrainConfig.frontRightName.get(), Constants.drivetrainConfig.backLeftName.get(),
                Constants.drivetrainConfig.backRightName.get(), "intake", Constants.SHOOTER_MOTOR_NAME,
                Constants.TRANSFER_MOTOR_NAME)), motors.keySet());
        assertMotor(motors, Constants.drivetrainConfig.frontLeftName.get(), "1", "1");
        assertMotor(motors, Constants.drivetrainConfig.frontRightName.get(), "173", "0");
        assertMotor(motors, Constants.drivetrainConfig.backLeftName.get(), "173", "3");
        assertMotor(motors, Constants.drivetrainConfig.backRightName.get(), "1", "0");
        assertMotor(motors, "intake", "1", "2");
        assertMotor(motors, Constants.SHOOTER_MOTOR_NAME, "1", "3");
        assertMotor(motors, Constants.TRANSFER_MOTOR_NAME, "173", "2");
        assertEquals(new HashSet<>(Arrays.asList(Constants.WEDGE_SERVO_NAME, "shotStopper")), servos.keySet());
        assertEquals("0", servos.get(Constants.WEDGE_SERVO_NAME).getAttribute("port"));
        assertEquals("1", servos.get("shotStopper").getAttribute("port"));
    }

    private static List<Element> children(Element parent) {
        List<Element> result = new ArrayList<>();
        NodeList nodes = parent.getChildNodes();
        for (int i = 0; i < nodes.getLength(); i++) {
            if (nodes.item(i) instanceof Element) result.add((Element) nodes.item(i));
        }
        return result;
    }

    private static void assertMotor(Map<String, Element> motors, String name, String address, String port) {
        Element motor = motors.get(name);
        assertNotNull("Missing motor " + name, motor);
        assertEquals(port, motor.getAttribute("port"));
        assertEquals(address, ((Element) motor.getParentNode()).getAttribute("port"));
    }
}
