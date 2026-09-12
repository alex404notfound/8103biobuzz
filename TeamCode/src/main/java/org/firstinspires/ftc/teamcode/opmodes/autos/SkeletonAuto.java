package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.robot.Alliance;

import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Proves paths + mirroring + scheduler end to end: drive one straight Bezier
 * line out from the corner start. Paths follow the DECODE idiom — a nested
 * Paths class holding public final PathChain fields, every control point routed
 * through transformed() so the whole auto mirrors (or shifts) in one place.
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
        public final PathChain driveOut;

        Paths() {
            driveOut = robot.drivetrain.pathBuilder()
                    .addPath(new BezierLine(transformed(7.5, 8.1), transformed(7.5, 32)))
                    // Linear heading interpolation: blend heading start -> end along the path.
                    // Endpoints are equal here, so the heading simply holds; linear is used
                    // as the common idiom, not because interpolation is needed for this leg.
                    .setLinearHeadingInterpolation(transformedHeading(90), transformedHeading(90))
                    .build();
        }
    }
}
