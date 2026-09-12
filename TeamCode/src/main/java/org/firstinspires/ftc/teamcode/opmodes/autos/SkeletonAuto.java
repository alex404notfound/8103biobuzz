package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.robot.Alliance;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.api.Paths.line;

/**
 * Example path + mirroring + command sequence. Coordinates are practice examples,
 * not a BIOBUZZ strategy. Route every control point through transformed().
 */
public abstract class SkeletonAuto extends AutoOpMode {
    private Paths paths;

    protected SkeletonAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected void onAutoInit() {
        paths = new Paths();
    }

    // Path geometry below is auto-local by design (this auto owns its coordinates);
    // shared field landmarks that other opmodes also need belong in FieldConstants.
    @Override
    protected Pose startingPose() {
        return transformed(7.5, 8.1).withHeading(transformedHeading(90));
    }

    @Override
    protected Command buildSequence() {
        return sequential(
                robot.drivetrain.followPath(paths.driveOut)
        );
    }

    private class Paths {
        public final Path driveOut;

        Paths() {
            driveOut = line(transformed(7.5, 8.1), transformed(7.5, 32))
                    .constant(transformedHeading(90));
        }
    }
}
