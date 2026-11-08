package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.field.Circle;
import com.bylazar.field.Line;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.zones.Point;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import dev.nextftc.robot.opmode.NextTeleop;
import org.firstinspires.ftc.teamcode.pedroPathing.MarrowConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.MarrowPlanner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.HiveBallTracker;
import org.firstinspires.ftc.teamcode.subsystems.HivePedroController;

import java.util.List;

/** Same held camera approach, with measured field-frame bounds and Weaver obstacle detours. */
@Disabled
@NextTeleop(name = "Hive Marrow Test", group = "Diagnostics")
public class HiveMarrowTest extends HivePedroTest {
    private MarrowPlanner planner;

    public HiveMarrowTest(Robot robot) { super(robot); }

    @Override protected Pose initialPose() {
        planner = MarrowConfig.createPlanner();
        Pose start = MarrowConfig.startingPose();
        planner.validatePose(start);
        return start;
    }

    @Override protected HivePedroController createApproach(HiveBallTracker tracker) {
        return new HivePedroController(robot.drivetrain, tracker, planner);
    }

    @Override protected void onInit() {
        super.onInit();
        // Capture the drawing settings with the planner; Panels edits apply on re-INIT.
        boolean obstacle = MarrowConfig.obstacleEnabled;
        double x = MarrowConfig.obstacleX, y = MarrowConfig.obstacleY, radius = MarrowConfig.obstacleRadiusInches;
        Style obstacleStyle = new Style("transparent", "#fb7185", .4);
        Style routeStyle = new Style("transparent", "#a78bfa", .2);
        robot.drivetrain.setFieldOverlay(canvas -> {
            if (obstacle) canvas.getItems().add(new Circle(x, y, radius, obstacleStyle));
            MarrowPlanner.Plan plan = approach.getPlannedRoute();
            if (plan == null) return;
            List<Point> points = plan.preview();
            for (int i = 1; i < points.size(); i++) {
                Point from = points.get(i - 1), to = points.get(i);
                canvas.getItems().add(new Line(from.getX(), from.getY(), to.getX(), to.getY(), routeStyle));
            }
        });
    }

    @Override protected void showPlannerTelemetry() {
        robot.telemetry.addLine("Marrow: measured field-frame start; configured keep-outs only. RT release/sticks/X stop.");
        MarrowPlanner.Plan plan = approach.getPlannedRoute();
        robot.telemetry.addData("Marrow route length upper bound in", plan == null ? "none" : plan.maximumLengthInches);
    }

    @Override protected void onStop() {
        try { super.onStop(); }
        finally { if (robot.drivetrain != null) robot.drivetrain.setFieldOverlay(null); }
    }
}
