package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

public class TrajectorySequenceRunner {
    private TrajectoryFollower follower;

    private final PIDFController turnController;
    private final NanoClock clock;

    private Pose2d lastPoseOnTurn = new Pose2d();
    private double turnStart = -1.0;
    private MotionProfile turnProfile;

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

        if (deltaTime >= currentTrajectorySequence.getDuration())
            currentTrajectorySequence = null;

        if (currentTrajectorySequence == null)
            return new DriveSignal();

        SequenceState currentSegment = currentTrajectorySequence.getCurrentState(deltaTime);

        boolean newTransition = trajectorySequenceLastSegmentIndex != currentSegment.getIndex();

        trajectorySequenceLastSegmentIndex = currentSegment.getIndex();

        if (currentSegment.getCurrentSegment() instanceof WaitSegment) {
            return new DriveSignal();
        } else if (currentSegment.getCurrentSegment() instanceof TurnSegment) {
            if (newTransition) {
                lastPoseOnTurn = poseEstimate;
                turnStart = now;

                turnProfile = ((TurnSegment) currentSegment.getCurrentSegment()).getMotionProfile();
            }

            double turnDeltaTime = now - turnStart;

            MotionState targetState = turnProfile.get(turnDeltaTime);

            turnController.setTargetPosition(targetState.getX());

            double correction = turnController.update(poseEstimate.getHeading());

            double targetOmega = targetState.getV();
            double targetAlpha = targetState.getA();

            return new DriveSignal(
                    new Pose2d(0, 0, targetOmega + correction),
                    new Pose2d(0, 0, targetAlpha)
            );
        } else if (currentSegment.getCurrentSegment() instanceof TrajectorySegment) {
            if (newTransition)
                follower.followTrajectory(((TrajectorySegment) currentSegment.getCurrentSegment()).getTrajectory());

            return follower.update(poseEstimate);
        }

        return new DriveSignal();
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
