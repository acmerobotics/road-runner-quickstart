package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Collections;

public class TrajectorySequenceRunner {
    private TrajectoryFollower follower;

    private final PIDFController turnController;

    private final NanoClock clock;

    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;

    private Pose2d lastPoseError = new Pose2d();
    private Pose2d lastPoseDesired = new Pose2d();

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

    public DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity) {
        if (currentSegmentIndex >= currentTrajectorySequence.size()) {
//            Log.i("trajectorysequence", "finished");

            currentTrajectorySequence = null;

            return new DriveSignal();
        }

        double now = clock.seconds();
        boolean newTransition = currentSegmentIndex != lastSegmentIndex;

        SequenceSegment segment = currentTrajectorySequence.get(currentSegmentIndex);

        if (newTransition) {
            currentSegmentStartTime = now;
            lastSegmentIndex = currentSegmentIndex;

            remainingMarkers.addAll(segment.getMarkers());
            Collections.sort(remainingMarkers, (trajectoryMarker, t1) -> Double.compare(trajectoryMarker.getTime(), t1.getTime()));
        }

        double deltaTime = now - currentSegmentStartTime;
//        Log.i("trajectorysequence", segment instanceof WaitSegment ? "waitsegment" : segment instanceof TurnSegment ? "turnsegment" : "trajectorysegment");
//        Log.i("trajectorysequence", Integer.toString(currentSegmentIndex));

        if (segment instanceof WaitSegment || segment instanceof TurnSegment) {
            if (deltaTime >= segment.getDuration()) {
                currentSegmentIndex++;

                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                return new DriveSignal();
            }

            while (remainingMarkers.size() > 0 && deltaTime > remainingMarkers.get(0).getTime()) {
                remainingMarkers.get(0).getCallback().onMarkerReached();
                remainingMarkers.remove(0);
            }
        }

        if (segment instanceof WaitSegment) {
            lastPoseError = new Pose2d();
            lastPoseDesired = segment.getStartPose();

            return new DriveSignal();
        } else if (segment instanceof TurnSegment) {
            MotionState targetState = ((TurnSegment) segment).getMotionProfile().get(deltaTime);

            turnController.setTargetPosition(targetState.getX());

            double correction = turnController.update(poseEstimate.getHeading());

            double targetOmega = targetState.getV();
            double targetAlpha = targetState.getA();

//            Log.i("trajectorysequence", "turn update");

            lastPoseError = new Pose2d(0, 0, turnController.getLastError());

            Pose2d startPose = segment.getStartPose();
            lastPoseDesired = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX());

            return new DriveSignal(
                    new Pose2d(0, 0, targetOmega + correction),
                    new Pose2d(0, 0, targetAlpha)
            );
        } else if (segment instanceof TrajectorySegment) {
            if (newTransition) {
//                Log.i("trajectorysequence", "trajectory transition");
                follower.followTrajectory(((TrajectorySegment) segment).getTrajectory());
//                Log.i("trajectorysequence", Double.toString(((TrajectorySegment) segment).getTrajectory().getProfile().duration()));
            }

            if (!follower.isFollowing()) {
                currentSegmentIndex++;

                return new DriveSignal();
            }

//            Log.i("trajectorysequence", "trajectory update");
//            Log.i("trajectorysequence", f.component1().toString());
            DriveSignal signal = follower.update(poseEstimate, poseVelocity);
            lastPoseError = follower.getLastError();
            lastPoseDesired = ((TrajectorySegment) segment).getTrajectory().get(deltaTime);

            return signal;
        }

        return new DriveSignal();
    }

    public Pose2d getLastPoseError() {
        return lastPoseError;
    }

    public void drawCurrentSequence(Canvas fieldOverlay) {
        if (currentTrajectorySequence == null) return;

        for (SequenceSegment segment : currentTrajectorySequence) {
            if (segment instanceof TrajectorySegment) {
                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");

                DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
            }
        }

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        DashboardUtil.drawRobot(fieldOverlay, lastPoseDesired);
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
