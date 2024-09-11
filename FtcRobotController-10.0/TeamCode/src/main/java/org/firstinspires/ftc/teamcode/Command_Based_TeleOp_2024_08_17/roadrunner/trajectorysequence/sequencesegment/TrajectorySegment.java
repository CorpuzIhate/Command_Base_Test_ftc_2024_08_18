package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Collections;

public final class TrajectorySegment extends SequenceSegment {
    private final Trajectory trajectory;

    public TrajectorySegment(Trajectory trajectory) {
        // Note: Markers are already stored in the `Trajectory` itself.
        // This class should not hold any markers
        super(trajectory.duration(), trajectory.start(), trajectory.end(), Collections.emptyList());
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() {
        return this.trajectory;
    }
}
