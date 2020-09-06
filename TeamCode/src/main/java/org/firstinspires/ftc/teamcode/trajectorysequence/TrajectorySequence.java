package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import java.util.List;

public class TrajectorySequence {
    private final List<SequenceSegment> sequenceSegments;
    private final double duration;
    private final List<TrajectoryMarker> markers;

    public TrajectorySequence(List<SequenceSegment> sequenceSegments, double duration, List<TrajectoryMarker> markers) {
        this.sequenceSegments = sequenceSegments;
        this.duration = duration;
        this.markers = markers;
    }

    public Pose2d get(Double time) {
        SequenceState currentState = getCurrentState(time);

        SequenceSegment currentSegment = currentState.getCurrentSegment();
        double segmentTime = currentState.getCurrentTime();

        if (currentSegment instanceof TrajectorySegment) {
            return ((TrajectorySegment) currentSegment).getTrajectory().get(segmentTime);
        } else if (currentSegment instanceof TurnSegment) {
            TurnSegment turnSegment = (TurnSegment) currentSegment;
            Pose2d newPose = turnSegment.getStartPose();
            newPose = turnSegment.getStartPose().copy(newPose.getX(), newPose.getY(), turnSegment.getMotionProfile().get(segmentTime).getX());

            return newPose;
        } else if(currentSegment instanceof WaitSegment) {
            return ((WaitSegment) currentSegment).getPose();
        } else {
            return new Pose2d();
        }
    }

    public Pose2d velocity(Double time) {
        SequenceState currentState = getCurrentState(time);

        SequenceSegment currentSegment = currentState.getCurrentSegment();
        double segmentTime = currentState.getCurrentTime();

        if (currentSegment instanceof TrajectorySegment) {
            return ((TrajectorySegment) currentSegment).getTrajectory().velocity(segmentTime);
        } else if (currentSegment instanceof TurnSegment) {
            TurnSegment turnSegment = (TurnSegment) currentSegment;
            Pose2d newPose = turnSegment.getStartPose();
            newPose = turnSegment.getStartPose().copy(newPose.getX(), newPose.getY(), turnSegment.getMotionProfile().get(segmentTime).getV());

            return newPose;
        } else if(currentSegment instanceof WaitSegment) {
            return new Pose2d();
        } else {
            return new Pose2d();
        }
    }

    public Pose2d acceleration(Double time) {
        SequenceState currentState = getCurrentState(time);

        SequenceSegment currentSegment = currentState.getCurrentSegment();
        double segmentTime = currentState.getCurrentTime();

        if (currentSegment instanceof TrajectorySegment) {
            return ((TrajectorySegment) currentSegment).getTrajectory().acceleration(segmentTime);
        } else if (currentSegment instanceof TurnSegment) {
            TurnSegment turnSegment = (TurnSegment) currentSegment;
            Pose2d newPose = turnSegment.getStartPose();
            newPose = turnSegment.getStartPose().copy(newPose.getX(), newPose.getY(), turnSegment.getMotionProfile().get(segmentTime).getA());

            return newPose;
        } else if(currentSegment instanceof WaitSegment) {
            return new Pose2d();
        } else {
            return new Pose2d();
        }
    }

    public Pose2d getStart() {
        return get(0.0);
    }

    public Pose2d getEnd() {
        return get(duration);
    }

    public List<SequenceSegment> getSequenceSegments() {
        return sequenceSegments;
    }

    public double getDuration() {
        return duration;
    }

    public List<TrajectoryMarker> getMarkers() {
        return markers;
    }

    public SequenceState getCurrentState(double time) {
        double currentTime = 0.0;

        int index = 0;
        for (SequenceSegment segment : sequenceSegments) {
            if (currentTime + segment.getDuration() > time) {
                double segmentTime = time - currentTime;

                return new SequenceState(segment, segmentTime, index);
            } else {
                currentTime += segment.getDuration();
            }

            index++;
        }

        return new SequenceState(null, 0.0, -1);
    }
}
