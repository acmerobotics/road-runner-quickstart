package org.firstinspires.ftc.teamcode.trajectorysequence;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

public final class SequenceState {
    private final SequenceSegment currentSegment;
    private final double currentTime;
    private final int index;

    public SequenceState(SequenceSegment segment, double time, int index) {
        currentSegment = segment;
        currentTime = time;
        this.index = index;
    }

    public SequenceSegment getCurrentSegment() {
        return currentSegment;
    }

    public double getCurrentTime() {
        return currentTime;
    }

    public int getIndex() {
        return index;
    }
}