package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

public class TrajectorySequenceRunner {
    private TrajectoryFollower follower;
    private final PIDFController turnController;
    private final NanoClock clock;

    private enum FollowMode {
        IDLE,
        TURN,
        TRAJECTORY,
        TRAJECTORY_SEQUENCE;
    }

    private enum TrajectorySequenceSegment {
        WAIT,
        TURN,
        TRAJECTORY
    }

    private FollowMode currentFollowMode = FollowMode.IDLE;

    private Pose2d lastPoseOnTurn = new Pose2d();
    private double turnStart = -1.0;
    private MotionProfile turnProfile;

    private TrajectorySequenceSegment currentTrajectorySequenceSegment = null;
    private TrajectorySequence currentTrajectorySequence = null;
    private double trajectorySequenceFollowingStart = -1.0;
    private int trajectorySequenceLastSegmentIndex = -1;

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        this.follower = follower;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        this.clock = NanoClock.system();
    }

    private void turnInternalAsync(Pose2d poseOnStart, MotionProfile turnProfile) {
        lastPoseOnTurn = poseOnStart;

        turnStart = clock.seconds();

        this.turnProfile = turnProfile;
    }

    public void turnAsync(double totalRotation, Pose2d poseOnStart, DriveConstraints constraints) {
        this.currentFollowMode = FollowMode.TURN;

        MotionProfile turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(poseOnStart.getHeading(), 0, 0, 0),
                new MotionState(poseOnStart.getHeading() + totalRotation, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        turnInternalAsync(poseOnStart, turnProfile);
    }

    public void turn(double totalRotation, Pose2d poseOnStart, DriveConstraints constraints, GetPoseEstimateCallback getPoseEstimateCallback) {
        turnAsync(totalRotation, poseOnStart, constraints);
        waitForIdle(getPoseEstimateCallback);
    }

    private void followTrajectoryInternalAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        currentFollowMode = FollowMode.TRAJECTORY;

        followTrajectoryInternalAsync(trajectory);
    }

    public void followTrajectory(Trajectory trajectory, GetPoseEstimateCallback getPoseEstimateCallback) {
        followTrajectoryAsync(trajectory);
        waitForIdle(getPoseEstimateCallback);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        currentFollowMode = FollowMode.TRAJECTORY_SEQUENCE;

        currentTrajectorySequence = trajectorySequence;
        trajectorySequenceFollowingStart = clock.seconds();
        trajectorySequenceLastSegmentIndex = -1;
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence, GetPoseEstimateCallback getPoseEstimateCallback) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle(getPoseEstimateCallback);
    }

    public DriveSignal update(Pose2d poseEstimate) {
        double now = clock.seconds();

        if (currentFollowMode == FollowMode.TRAJECTORY_SEQUENCE && currentTrajectorySequence != null) {
            SequenceState currentSequence;

            if (now - trajectorySequenceFollowingStart >= currentTrajectorySequence.getDuration()) {
                currentFollowMode = FollowMode.IDLE;
            } else {
                currentSequence = currentTrajectorySequence.getCurrentState(now - trajectorySequenceFollowingStart);

                if (trajectorySequenceLastSegmentIndex != currentSequence.getIndex()) {
                    if (currentSequence.getCurrentSegment() instanceof WaitSegment) {
                        currentTrajectorySequenceSegment = TrajectorySequenceSegment.WAIT;
                    } else if (currentSequence.getCurrentSegment() instanceof TurnSegment) {
                        currentTrajectorySequenceSegment = TrajectorySequenceSegment.TURN;

                        TurnSegment currentSegment = (TurnSegment) currentSequence.getCurrentSegment();

                        turnInternalAsync(currentSegment.getStartPose(), currentSegment.getMotionProfile());
                    } else if (currentSequence.getCurrentSegment() instanceof TrajectorySegment) {
                        currentTrajectorySequenceSegment = TrajectorySequenceSegment.TRAJECTORY;

                        followTrajectoryInternalAsync(((TrajectorySegment) currentSequence.getCurrentSegment()).getTrajectory());
                    }
                }

                trajectorySequenceLastSegmentIndex = currentSequence.getIndex();
            }
        }

        if (currentFollowMode == FollowMode.IDLE || (currentFollowMode == FollowMode.TRAJECTORY_SEQUENCE && currentTrajectorySequenceSegment == TrajectorySequenceSegment.WAIT)) {
            return new DriveSignal();
        } else if (currentFollowMode == FollowMode.TURN || (currentFollowMode == FollowMode.TRAJECTORY_SEQUENCE && currentTrajectorySequenceSegment == TrajectorySequenceSegment.TURN)) {
            double timeDelta = now - turnStart;

            MotionState targetState = turnProfile.get(timeDelta);

            turnController.setTargetPosition(targetState.getX());

            double correction = turnController.update(poseEstimate.getHeading());

            double targetOmega = targetState.getV();
            double targetAlpha = targetState.getA();

            if (timeDelta >= turnProfile.duration()) {
                if (currentFollowMode == FollowMode.TURN) {
                    currentFollowMode = FollowMode.IDLE;

                    return new DriveSignal();
                } else if (currentFollowMode == FollowMode.TRAJECTORY_SEQUENCE) {
                    currentTrajectorySequenceSegment = TrajectorySequenceSegment.WAIT;

                    return new DriveSignal();
                }
            }

            return new DriveSignal(
                    new Pose2d(0, 0, targetOmega + correction),
                    new Pose2d(0, 0, targetAlpha)
            );
        } else if (currentFollowMode == FollowMode.TRAJECTORY || (currentFollowMode == FollowMode.TRAJECTORY_SEQUENCE && currentTrajectorySequenceSegment == TrajectorySequenceSegment.TRAJECTORY)) {
            if (!follower.isFollowing()) {
                if (currentFollowMode == FollowMode.TRAJECTORY) {
                    currentFollowMode = FollowMode.IDLE;

                    return new DriveSignal();
                } else if (currentFollowMode == FollowMode.TRAJECTORY_SEQUENCE) {
                    currentTrajectorySequenceSegment = TrajectorySequenceSegment.WAIT;

                    return new DriveSignal();
                }
            }

            return follower.update(poseEstimate);
        }

        return new DriveSignal();
    }

    public void waitForIdle(GetPoseEstimateCallback getPoseEstimateCallback) {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update(getPoseEstimateCallback.getPoseEstimate());
        }
    }

    public boolean isBusy() {
        return currentFollowMode != FollowMode.IDLE;
    }

    public interface GetPoseEstimateCallback {
        Pose2d getPoseEstimate();
    }
}
