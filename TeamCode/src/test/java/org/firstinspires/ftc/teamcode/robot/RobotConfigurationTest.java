package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
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

    @Test public void oneConfigurationProvidesEveryMotorPinpointAndTheControlHubServos() throws Exception {
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
        Map<String, Element> pinpoints = new HashMap<>();
        String pinpointTag = GoBildaPinpointDriver.class.getAnnotation(DeviceProperties.class).xmlTag();
        List<Element> modules = children(portal);
        assertEquals(2, modules.size());
        for (Element module : modules) {
            assertEquals("LynxModule", module.getTagName());
            assertTrue("Duplicate hub address", moduleAddresses.add(module.getAttribute("port")));
            Set<String> ports = new HashSet<>();
            for (Element device : children(module)) {
                if (pinpointTag.equals(device.getTagName())) {
                    assertTrue("Duplicate I2C device", ports.add("i2c:" + device.getAttribute("bus")
                            + ":" + device.getAttribute("port")));
                    assertNull(pinpoints.put(device.getAttribute("name"), device));
                } else if ("Servo".equals(device.getTagName())) {
                    assertTrue("Duplicate servo port", ports.add("servo:" + device.getAttribute("port")));
                    assertNull(servos.put(device.getAttribute("name"), device));
                } else {
                    assertTrue(device.getTagName().endsWith("Motor"));
                    assertTrue("Duplicate motor port", ports.add("motor:" + device.getAttribute("port")));
                    assertNull(motors.put(device.getAttribute("name"), device));
                }
            }
        }
        assertEquals(new HashSet<>(Arrays.asList("173", "1")), moduleAddresses);
        assertEquals(7, motors.size());
        assertEquals(new HashSet<>(Arrays.asList(Constants.drivetrainConfig.frontLeftName.get(),
                Constants.drivetrainConfig.frontRightName.get(), Constants.drivetrainConfig.backLeftName.get(),
                Constants.drivetrainConfig.backRightName.get(), Constants.INTAKE_MOTOR_NAME,
                Constants.SHOOTER_MOTOR_NAME, Constants.SECOND_SHOOTER_MOTOR_NAME)), motors.keySet());
        assertHubPort(motors, Constants.drivetrainConfig.frontLeftName.get(), "173", "0");
        assertHubPort(motors, Constants.drivetrainConfig.frontRightName.get(), "1", "2");
        assertHubPort(motors, Constants.drivetrainConfig.backLeftName.get(), "173", "1");
        assertHubPort(motors, Constants.drivetrainConfig.backRightName.get(), "1", "1");
        assertHubPort(motors, Constants.INTAKE_MOTOR_NAME, "1", "0");
        assertHubPort(motors, Constants.SHOOTER_MOTOR_NAME, "1", "3");
        assertHubPort(motors, Constants.SECOND_SHOOTER_MOTOR_NAME, "173", "2");
        assertEquals(3, servos.size());
        assertHubPort(servos, Constants.WEDGE_SERVO_NAME, "173", "0");
        assertHubPort(servos, "shotStopper", "173", "2");
        assertHubPort(servos, Constants.TURRET_SERVO_NAME, "173", "4");
        assertEquals(new HashSet<>(Arrays.asList(Constants.localizerConfig.name.get())), pinpoints.keySet());
        Element pinpoint = pinpoints.get(Constants.localizerConfig.name.get());
        assertEquals("173", ((Element) pinpoint.getParentNode()).getAttribute("port"));
        assertEquals("1", pinpoint.getAttribute("port"));
        assertEquals("1", pinpoint.getAttribute("bus"));
    }

    private static List<Element> children(Element parent) {
        List<Element> result = new ArrayList<>();
        NodeList nodes = parent.getChildNodes();
        for (int i = 0; i < nodes.getLength(); i++) {
            if (nodes.item(i) instanceof Element) result.add((Element) nodes.item(i));
        }
        return result;
    }

    private static void assertHubPort(Map<String, Element> devices, String name, String address, String port) {
        Element device = devices.get(name);
        assertNotNull("Missing device " + name, device);
        assertEquals(port, device.getAttribute("port"));
        assertEquals(address, ((Element) device.getParentNode()).getAttribute("port"));
    }
}
