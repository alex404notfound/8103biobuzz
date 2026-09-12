package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;
import org.junit.Test;
import static org.junit.Assert.*;

public class VisionFrameTest {
    @Test public void convertsMetersAndRotatesAroundTranslatedOrigin() {
        Pose pose = new VisionFrame(10, 20, 90).toPedro(1, 0, -90);
        assertEquals(10, pose.getX(), 1e-9);
        assertEquals(20 + 100 / 2.54, pose.getY(), 1e-9);
        assertEquals(0, pose.getHeading(), 1e-9);
    }

    @Test public void outgoingHeadingIsTheInverseAtEveryCardinalAndAcrossWrap() {
        VisionFrame frame = new VisionFrame(70.75, 70.75, 90);
        for (double cameraHeading : new double[]{-179, -90, 0, 90, 179, 270}) {
            double pedroHeading = frame.toPedro(0, 0, cameraHeading).getHeading();
            double difference = Math.toRadians(frame.toCameraHeading(pedroHeading) - cameraHeading);
            assertEquals(0, Math.atan2(Math.sin(difference), Math.cos(difference)), 1e-9);
        }
    }

    @Test public void rejectsNonfiniteCalibrationAndMeasurements() {
        assertThrows(IllegalArgumentException.class, () -> new VisionFrame(Double.NaN, 0, 0));
        VisionFrame frame = new VisionFrame(0, 0, 0);
        assertThrows(IllegalArgumentException.class, () -> frame.toPedro(0, Double.POSITIVE_INFINITY, 0));
    }
}
