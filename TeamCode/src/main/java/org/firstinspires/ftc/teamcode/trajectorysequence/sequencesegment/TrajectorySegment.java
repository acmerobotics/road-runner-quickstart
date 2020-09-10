package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

public final class TrajectorySegment extends SequenceSegment {
    private final Trajectory trajectory;

    public TrajectorySegment(Trajectory trajectory) {
        super(trajectory.duration());
        this.trajectory = trajectory;
    }

    public final Trajectory getTrajectory() {
        return this.trajectory;
    }
}
