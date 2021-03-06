package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Collections;

@Config
public class TrajectorySequenceRunner {
    public static String COLOR_INACTIVE_TRAJECTORY = "#4caf507a";
    public static String COLOR_INACTIVE_TURN = "#7c4dff7a";
    public static String COLOR_INACTIVE_WAIT = "#dd2c007a";

    public static String COLOR_ACTIVE_TRAJECTORY = "#4CAF50";
    public static String COLOR_ACTIVE_TURN = "#7c4dff";
    public static String COLOR_ACTIVE_WAIT = "#dd2c00";

    private final TrajectoryFollower follower;

    private final PIDFController turnController;

    private final NanoClock clock;

    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;

    private Pose2d lastPoseError = new Pose2d();

    ArrayList<TrajectoryMarker> remainingMarkers = new ArrayList<>();

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        this.follower = follower;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        this.clock = NanoClock.system();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
        currentSegmentStartTime = clock.seconds();
        currentSegmentIndex = 0;
        lastSegmentIndex = -1;
    }

    public DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity, Canvas fieldOverlay) {
        Pose2d targetPose = new Pose2d();
        DriveSignal driveSignal = new DriveSignal();

        if (currentTrajectorySequence != null) {
            for (SequenceSegment segment : currentTrajectorySequence) {
                if (segment instanceof TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY);

                    DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
                } else if (segment instanceof TurnSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setFill(COLOR_INACTIVE_TURN);
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2);
                } else if (segment instanceof WaitSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
                }
            }

            if (currentSegmentIndex >= currentTrajectorySequence.size()) {
                targetPose = currentTrajectorySequence.getLast().getEndPose();
                driveSignal = new DriveSignal();

                currentTrajectorySequence = null;
            } else {
                double now = clock.seconds();
                boolean isNewTransition = currentSegmentIndex != lastSegmentIndex;

                SequenceSegment currentSegment = currentTrajectorySequence.get(currentSegmentIndex);

                if (isNewTransition) {
                    currentSegmentStartTime = now;
                    lastSegmentIndex = currentSegmentIndex;

                    remainingMarkers.addAll(currentSegment.getMarkers());
                    Collections.sort(remainingMarkers, (trajectoryMarker, t1) -> Double.compare(trajectoryMarker.getTime(), t1.getTime()));
                }

                double deltaTime = now - currentSegmentStartTime;

                if (currentSegment instanceof WaitSegment) {
                    lastPoseError = new Pose2d();

                    targetPose = currentSegment.getStartPose();
                    driveSignal = new DriveSignal();

                    Pose2d pose = currentSegment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_ACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
                } else if (currentSegment instanceof TurnSegment) {
                    MotionState targetState = ((TurnSegment) currentSegment).getMotionProfile().get(deltaTime);

                    turnController.setTargetPosition(targetState.getX());

                    double correction = turnController.update(poseEstimate.getHeading());

                    double targetOmega = targetState.getV();
                    double targetAlpha = targetState.getA();

                    lastPoseError = new Pose2d(0, 0, turnController.getLastError());

                    Pose2d startPose = currentSegment.getStartPose();
                    targetPose = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX());

                    driveSignal = new DriveSignal(
                            new Pose2d(0, 0, targetOmega + correction),
                            new Pose2d(0, 0, targetAlpha)
                    );

                    Pose2d pose = currentSegment.getStartPose();

                    fieldOverlay.setFill(COLOR_ACTIVE_TURN);
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 3);
                } else if (currentSegment instanceof TrajectorySegment) {
                    Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                    if (isNewTransition)
                        follower.followTrajectory(currentTrajectory);

                    if (!follower.isFollowing()) {
                        currentSegmentIndex++;

                        driveSignal = new DriveSignal();
                    } else {
                        driveSignal = follower.update(poseEstimate, poseVelocity);
                        lastPoseError = follower.getLastError();
                    }

                    targetPose = currentTrajectory.get(deltaTime);

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY);

                    DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory.getPath());
                }

                if (currentSegment instanceof WaitSegment || currentSegment instanceof TurnSegment) {
                    if (deltaTime >= currentSegment.getDuration()) {
                        currentSegmentIndex++;

                        for (TrajectoryMarker marker : remainingMarkers) {
                            marker.getCallback().onMarkerReached();
                        }

                        remainingMarkers.clear();

                        driveSignal = new DriveSignal();
                    }

                    while (remainingMarkers.size() > 0 && deltaTime > remainingMarkers.get(0).getTime()) {
                        remainingMarkers.get(0).getCallback().onMarkerReached();
                        remainingMarkers.remove(0);
                    }
                }
            }
        }

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        DashboardUtil.drawRobot(fieldOverlay, targetPose);

        return driveSignal;
    }

    public Pose2d getLastPoseError() {
        return lastPoseError;
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
