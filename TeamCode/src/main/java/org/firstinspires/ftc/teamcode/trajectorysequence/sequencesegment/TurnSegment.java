package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;

public final class TurnSegment extends SequenceSegment {
    private final Pose2d startPose;
    private final double totalRotation;
    private final MotionProfile motionProfile;

    public TurnSegment(Pose2d startPose, double totalRotation, MotionProfile motionProfile, double duration) {
        super(duration);

        this.startPose = startPose;
        this.totalRotation = totalRotation;
        this.motionProfile = motionProfile;
    }

    public final Pose2d getStartPose() {
        return this.startPose;
    }

    public final double getTotalRotation() {
        return this.totalRotation;
    }

    public final MotionProfile getMotionProfile() {
        return this.motionProfile;
    }
}
