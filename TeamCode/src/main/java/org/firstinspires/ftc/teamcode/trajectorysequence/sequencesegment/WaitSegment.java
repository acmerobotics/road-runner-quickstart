package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public final class WaitSegment extends SequenceSegment {
    private final Pose2d pose;
    private final double seconds;

    public WaitSegment(Pose2d pose, double seconds) {
        super(seconds);

        this.pose = pose;
        this.seconds = seconds;
    }

    public final Pose2d getPose() {
        return this.pose;
    }

    public final double getSeconds() {
        return this.seconds;
    }
}
