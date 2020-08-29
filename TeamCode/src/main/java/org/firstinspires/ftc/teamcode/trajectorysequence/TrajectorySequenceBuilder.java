package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.SpatialMarker;
import com.acmerobotics.roadrunner.trajectory.TemporalMarker;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import java.util.ArrayList;
import java.util.List;

import kotlin.jvm.functions.Function1;

public class TrajectorySequenceBuilder {
    private final Pose2d startPose;
    private final double startTangent;
    private final DriveConstraints constraints;
    private final double resolution;

    private final List<SequenceSegment> sequenceSegments;

    private final List<TemporalMarker> temporalMarkers;
    private final List<DisplacementMarker> displacementMarkers;
    private final List<SpatialMarker> spatialMarkers;

    private Pose2d lastPose;
    private double lastTangent;

    private double tangentOffset;

    private TrajectoryBuilder currentTrajectoryBuilder;

    private double currentDuration;
    private double currentDisplacement;

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            double startTangent,
            DriveConstraints constraints,
            double resolution
    ) {
        this.startPose = startPose;
        this.startTangent = startTangent;
        this.constraints = constraints;
        this.resolution = resolution;

        sequenceSegments = new ArrayList<>();

        temporalMarkers = new ArrayList<>();
        displacementMarkers = new ArrayList<>();
        spatialMarkers = new ArrayList<>();

        lastPose = startPose;
        lastTangent = startTangent;

        tangentOffset = 0.0;

        currentTrajectoryBuilder = null;

        currentDuration = 0.0;
        currentDisplacement = 0.0;
    }

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            double startTangent,
            DriveConstraints constraints
    ) {
        this(startPose, startTangent, constraints, 0.25);
    }

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            DriveConstraints constraints,
            double resolution
    ) {
        this(startPose, startPose.getHeading(), constraints, resolution);
    }

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            DriveConstraints constraints
    ) {
        this(startPose, startPose.getHeading(), constraints, 0.25);
    }

    private TrajectorySequenceBuilder addPath(AddPathCallback callback) {
        if (currentTrajectoryBuilder == null) newPath();

        try {
            callback.run();
        } catch (PathContinuityViolationException e) {
            newPath();
            callback.run();
        }

        Trajectory builtTraj = currentTrajectoryBuilder.build();

        lastPose = builtTraj.end();
        currentDuration += builtTraj.duration();
        currentDisplacement += builtTraj.getPath().length();

        return this;
    }

    public void addDisplacementMarker(Function1<? super Double, Double> displacement, MarkerCallback callback) {
        displacementMarkers.add(new DisplacementMarker(displacement, callback));
    }

    public TrajectorySequenceBuilder turn(Double angle) {
        pushPath();

        MotionProfile turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lastPose.getHeading(), 0.0, 0.0, 0.0),
                new MotionState(lastPose.getHeading() + angle, 0.0, 0.0, 0.0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        sequenceSegments.add(new TurnSegment(lastPose, angle, turnProfile, turnProfile.duration()));

        lastPose = lastPose.copy(lastPose.getX(), lastPose.getY(), lastPose.getHeading() + angle);

        currentDuration += turnProfile.duration();

        return this;
    }

    public TrajectorySequenceBuilder waitSeconds(Double seconds) {
        pushPath();
        sequenceSegments.add(new WaitSegment(lastPose, seconds));

        currentDuration += seconds;
        return this;
    }

    private void pushPath() {
        if (currentTrajectoryBuilder != null) {
            Trajectory builtTraj = currentTrajectoryBuilder.build();
            sequenceSegments.add(new TrajectorySegment(builtTraj, builtTraj.duration()));
        }
    }

    private void newPath() {
        if (currentTrajectoryBuilder != null)
            pushPath();

        currentTrajectoryBuilder = new TrajectoryBuilder(lastPose, Angle.norm(lastPose.getHeading() + tangentOffset), constraints);
    }

    // Required for kotlin translation
    // No java equivalent
    private interface AddPathCallback {
        void run();
    }
}
