package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.ArrayList;

public final class TrajectorySegment extends SequenceSegment {
    private final Trajectory trajectory;

    public TrajectorySegment(Trajectory trajectory) {
        super(trajectory.duration(), trajectory.start(), trajectory.end(), new ArrayList<>());
        this.trajectory = trajectory;
    }

    public final Trajectory getTrajectory() {
        return this.trajectory;
    }
}
