package org.firstinspires.ftc.teamcode.math;

import java.util.ArrayList;
import java.util.List;

/**
 * An immutable, continuous-position/velocity plan to a stationary, unwrapped angle.
 * Units are turret degrees and seconds. Acceleration changes at phase boundaries.
 * Replan from a sampled state to change the goal without jumping position or velocity.
 *
 * If the initial velocity cannot stop before the goal, the profile brakes, passes
 * the goal, and returns. Extents include that unavoidable excursion so a caller
 * can reject plans that would cross a configured travel limit. The profile describes
 * desired movement; its constraints are not a guarantee of actual stopping distance.
 */
public final class TurretMotionProfile {
    public static final class State {
        public final double position, velocity, acceleration;

        private State(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    private final List<Phase> phases;
    private final double goalPosition, duration, minPosition, maxPosition;

    public TurretMotionProfile(double startPosition, double startVelocity, double goalPosition,
                               double maximumVelocity, double maximumAcceleration) {
        finite(startPosition); finite(startVelocity); finite(goalPosition);
        positive(maximumVelocity); positive(maximumAcceleration);
        this.goalPosition = goalPosition;
        Planner plan = new Planner(startPosition, startVelocity);
        double minimum = Math.min(startPosition, goalPosition);
        double maximum = Math.max(startPosition, goalPosition);
        double distance = goalPosition - startPosition;
        finite(distance);

        if (startVelocity != 0) {
            double stopTime = Math.abs(startVelocity) / maximumAcceleration;
            double stopPosition = startPosition + startVelocity * stopTime / 2;
            finite(stopTime); finite(stopPosition);
            double direction = Math.signum(startVelocity);
            if (distance * direction <= 0 || (stopPosition - goalPosition) * direction > 0) {
                // Braking while moving away, or when a nearer goal cannot be reached at rest.
                plan.append(-direction * maximumAcceleration, stopTime, 0);
                minimum = Math.min(minimum, plan.position);
                maximum = Math.max(maximum, plan.position);
            } else if (Math.abs(startVelocity) > maximumVelocity) {
                // A measured/replanned overspeed state cannot be clamped instantaneously.
                plan.append(-direction * maximumAcceleration,
                        (Math.abs(startVelocity) - maximumVelocity) / maximumAcceleration,
                        direction * maximumVelocity);
            }
        }

        distance = goalPosition - plan.position;
        finite(distance);
        if (distance != 0) {
            double direction = Math.signum(distance);
            double length = Math.abs(distance);
            double initialSpeed = direction * plan.velocity;
            double peakSquared = maximumAcceleration * length + initialSpeed * initialSpeed / 2;
            finite(peakSquared);
            double peakSpeed = Math.min(maximumVelocity,
                    Math.max(initialSpeed, Math.sqrt(peakSquared)));
            double accelerateTime = Math.max(0, (peakSpeed - initialSpeed) / maximumAcceleration);
            double decelerateTime = peakSpeed / maximumAcceleration;
            double accelerateDistance = (initialSpeed + peakSpeed) * accelerateTime / 2;
            double decelerateDistance = peakSpeed * decelerateTime / 2;
            // A triangle has no cruise; subtracting its rounded distances can invent a
            // sub-ulp phase and a spurious nonzero acceleration discontinuity.
            double cruiseDistance = peakSpeed < maximumVelocity ? 0
                    : Math.max(0, length - accelerateDistance - decelerateDistance);
            if (cruiseDistance <= 8 * Math.ulp(length)) cruiseDistance = 0;
            plan.append(direction * maximumAcceleration, accelerateTime, direction * peakSpeed);
            plan.append(0, cruiseDistance / peakSpeed, direction * peakSpeed);
            plan.append(-direction * maximumAcceleration, decelerateTime, 0);
        }

        phases = plan.phases;
        duration = plan.time;
        // Every phase is monotonic between start, an optional braking stop, and goal.
        // Use the exact requested endpoint, not its rounded integration equivalent.
        minPosition = minimum;
        maxPosition = maximum;
    }

    /** At and after duration, returns exactly the requested goal with zero velocity/acceleration. */
    public State sample(double elapsedSeconds) {
        finite(elapsedSeconds);
        if (elapsedSeconds < 0) throw new IllegalArgumentException("Profile sample time must be nonnegative");
        if (elapsedSeconds >= duration) return new State(goalPosition, 0, 0);
        for (Phase phase : phases) {
            if (elapsedSeconds < phase.startTime + phase.seconds) {
                return phase.sample(Math.max(0, elapsedSeconds - phase.startTime));
            }
        }
        return new State(goalPosition, 0, 0);
    }

    public double duration() { return duration; }
    public double minPosition() { return minPosition; }
    public double maxPosition() { return maxPosition; }

    private static final class Phase {
        final double startTime, seconds, position, velocity, acceleration;

        Phase(double startTime, double seconds, double position, double velocity, double acceleration) {
            this.startTime = startTime;
            this.seconds = seconds;
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        State sample(double time) {
            return new State(position + (velocity + acceleration * time / 2) * time,
                    velocity + acceleration * time, acceleration);
        }
    }

    private static final class Planner {
        final List<Phase> phases = new ArrayList<>();
        double position, velocity, time;

        Planner(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        void append(double acceleration, double seconds, double endVelocity) {
            finite(seconds);
            if (seconds < 0) throw new IllegalArgumentException("Negative profile phase duration");
            if (seconds == 0) return;
            Phase phase = new Phase(time, seconds, position, velocity, acceleration);
            State end = phase.sample(seconds);
            finite(end.position); finite(end.velocity); finite(endVelocity);
            double endTime = time + seconds;
            finite(endTime);
            if (endTime <= time) throw new IllegalArgumentException("Profile phase is below time precision");
            phases.add(phase);
            position = end.position;
            velocity = endVelocity; // Retain exact zero/peak speed at phase boundaries.
            time = endTime;
        }
    }

    private static void finite(double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Profile values must be finite");
    }

    private static void positive(double value) {
        finite(value);
        if (value <= 0) throw new IllegalArgumentException("Profile constraints must be positive");
    }
}
