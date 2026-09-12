package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/** Bounded Pedro 3 path examples; each phase uses the checked drivetrain and actual path completion. */
public final class Tests extends Procedure {
    public enum Test { LINE, CURVE, HOLD, LOCALIZATION }
    public Tests() { super("Path and localization tests", "Verify measured Foresight parameters with short paths, or inspect manual localization."); }

    @Override public void run() throws InterruptedException {
        Inputs inputs = inputs("Choose a test", "Start with a short clear path. LINE travels out and back; CURVE ends diagonally away from the start. HOLD lasts 5 seconds.");
        Inputs.Field<Test> selected = inputs.e("Test", Test.class).withDefault(Test.LOCALIZATION);
        Inputs.Field<Double> distance = inputs.d("Distance (inches)").withDefault(24.0);
        awaitInputs(inputs);
        CalibrationValues.range(distance.get(), 6, 72, "Test distance");
        if (selected.get() != Test.LOCALIZATION && !Constants.foresightTuned) {
            abort("Finish Foresight calibration, enter all measured values, and set Constants.foresightTuned before path tests.");
            return;
        }
        Pose pose = runOpMode(new PathTest(selected.get(), distance.get()));
        result("Final measured pose", pose);
        result("Next step", "Compare measured motion with the marked field positions before using autonomous paths.");
    }
}

final class PathTest extends SafeTuningOpMode<Pose> {
    private final Tests.Test selected;
    private final double distance;
    PathTest(Tests.Test selected, double distance) {
        super(selected + " test", "The chosen test is " + selected + " with distance " + distance
                + " inches. Localization is manual; release/press A to finish that test. B/STOP always aborts.", true);
        this.selected = selected;
        this.distance = distance;
    }
    @Override protected long timeoutNanos() {
        return selected == Tests.Test.LOCALIZATION ? 120_000_000_000L : 30_000_000_000L;
    }
    @Override protected Pose runSafely() throws InterruptedException {
        Localizer localizer = localizer();
        localizer.setPose(Pose.zero());
        if (selected == Tests.Test.LOCALIZATION) {
            boolean released = false;
            while (true) {
                localizer.update();
                telemetry.addData("Pose", localizer.pose());
                telemetry.addData("Velocity (in/s, rad/s)", localizer.velocity());
                telemetry.addLine("Move by hand. Release/press A to finish. B/STOP aborts.");
                telemetry.update();
                if (!gamepad1.a) released = true;
                if (released && gamepad1.a) return localizer.pose();
                Thread.sleep(10);
            }
        }
        Follower follower = follower();
        if (selected == Tests.Test.HOLD) {
            follower.hold(Pose.zero());
            long end = System.nanoTime() + 5_000_000_000L;
            while (System.nanoTime() < end) { follower.update(); Thread.sleep(10); }
        } else if (selected == Tests.Test.CURVE) {
            followAndWait(follower, curve(Pose.zero(), new Pose(distance, 0), new Pose(distance, distance)).constant(0));
        } else {
            followAndWait(follower, line(Pose.zero(), new Pose(distance, 0, 0)).constant(0));
            followAndWait(follower, line(new Pose(distance, 0, 0), Pose.zero()).constant(0));
        }
        follower.stop();
        follower.drivetrain.stop(true);
        return localizer.pose();
    }

    private void followAndWait(Follower follower, Path path) throws InterruptedException {
        follower.follow(path);
        do {
            follower.update();
            telemetry.addData("Pose", follower.pose());
            telemetry.update();
            Thread.sleep(10);
        } while (!follower.idle() && (!follower.holding() || follower.isBusy()));
    }
}
