package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

public abstract class SequenceSegment {
    private final double duration;

    protected SequenceSegment(double duration) {
        this.duration = duration;
    }

    public double getDuration() {
        return this.duration;
    }
}
