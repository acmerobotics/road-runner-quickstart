package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

public final class TurnSegment extends SequenceSegment {
    private final double totalRotation;
    private final MotionProfile motionProfile;

    public TurnSegment(Pose2d startPose, double totalRotation, MotionProfile motionProfile, List<TrajectoryMarker> markers) {
        super(
                motionProfile.duration(),
                startPose,
                startPose.copy(
                        startPose.getX(),
                        startPose.getY(),
                        startPose.getHeading() + totalRotation
                ),
                markers
        );

        this.totalRotation = totalRotation;
        this.motionProfile = motionProfile;
    }

    public final double getTotalRotation() {
        return this.totalRotation;
    }

    public final MotionProfile getMotionProfile() {
        return this.motionProfile;
    }
}
