package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

public class TrajectorySequenceRunner {
    private TrajectoryFollower follower;

    private final PIDFController turnController;
    private final NanoClock clock;

    private TrajectorySequence currentTrajectorySequence = null;
    private double trajectorySequenceFollowingStart = -1.0;
    private int trajectorySequenceLastSegmentIndex = -1;

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        this.follower = follower;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        this.clock = NanoClock.system();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
        trajectorySequenceFollowingStart = clock.seconds();
        trajectorySequenceLastSegmentIndex = -1;
    }

    public DriveSignal update(Pose2d poseEstimate) {
        double now = clock.seconds();
        double deltaTime = now - trajectorySequenceFollowingStart;

        if (deltaTime >= currentTrajectorySequence.duration())
            currentTrajectorySequence = null;

        if (currentTrajectorySequence == null)
            return new DriveSignal();

        SequenceSegment segment = null;
        double segmentOffsetTime = 0;
        int currentSegmentIndex = -1;

        double currentAccumulatedTime = 0;
        for (int i = 0; i < currentTrajectorySequence.size(); i++) {
            SequenceSegment seg = currentTrajectorySequence.get(i);

            if (currentAccumulatedTime + seg.getDuration() > deltaTime) {
                segmentOffsetTime = deltaTime - currentAccumulatedTime;
                segment = seg;

                currentSegmentIndex = i;

                break;
            } else {
                currentAccumulatedTime += seg.getDuration();
            }
        }

        boolean newTransition = trajectorySequenceLastSegmentIndex != currentSegmentIndex;

        trajectorySequenceLastSegmentIndex = currentSegmentIndex;

        if (segment instanceof WaitSegment) {
            return new DriveSignal();
        } else if (segment instanceof TurnSegment) {
            MotionState targetState = ((TurnSegment) segment).getMotionProfile().get(segmentOffsetTime);

            turnController.setTargetPosition(targetState.getX());

            double correction = turnController.update(poseEstimate.getHeading());

            double targetOmega = targetState.getV();
            double targetAlpha = targetState.getA();

            return new DriveSignal(
                    new Pose2d(0, 0, targetOmega + correction),
                    new Pose2d(0, 0, targetAlpha)
            );
        } else if (segment instanceof TrajectorySegment) {
            if (newTransition)
                follower.followTrajectory(((TrajectorySegment) segment).getTrajectory());

            return follower.update(poseEstimate);
        }

        return new DriveSignal();
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
