package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.LinkedList;

public class TrajectorySequence extends LinkedList<SequenceSegment> {
    public Pose2d start() {
        if (!this.isEmpty())
            return this.get(0).getStartPose();
        else
            return new Pose2d();
    }

    public Pose2d end() {
        if (!this.isEmpty())
            return this.get(this.size() - 1).getEndPose();
        else
            return new Pose2d();
    }

    public double duration() {
        double total = 0.0;

        for (SequenceSegment segment : this) {
            total += segment.getDuration();
        }

        return total;
    }
}
