package org.firstinspires.ftc.teamcode.trajectorysequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
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
    private final double resolution;

    private DriveConstraints constraints;

    private final TrajectorySequence sequenceSegments;

    private final List<TemporalMarker> temporalMarkers;
    private final List<DisplacementMarker> displacementMarkers;
    private final List<SpatialMarker> spatialMarkers;

    private Pose2d lastPose;

    private double tangentOffset;

    private boolean setAbsoluteTangent;
    private double absoluteTangent;

    private TrajectoryBuilder currentTrajectoryBuilder;

    private double currentDuration;
    private double currentDisplacement;

    private double lastDurationTraj;
    private double lastDisplacementTraj;

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            Double startTangent,
            DriveConstraints constraints,
            double resolution
    ) {
        this.constraints = constraints;
        this.resolution = resolution;

        sequenceSegments = new TrajectorySequence();

        temporalMarkers = new ArrayList<>();
        displacementMarkers = new ArrayList<>();
        spatialMarkers = new ArrayList<>();

        lastPose = startPose;

        tangentOffset = 0.0;

        setAbsoluteTangent = (startTangent != null);
        absoluteTangent = startTangent != null ? startTangent : 0.0;

        currentTrajectoryBuilder = null;

        currentDuration = 0.0;
        currentDisplacement = 0.0;

        lastDurationTraj = 0.0;
        lastDisplacementTraj = 0.0;
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
        this(startPose, null, constraints, resolution);
    }

    public TrajectorySequenceBuilder(
            Pose2d startPose,
            DriveConstraints constraints
    ) {
        this(startPose, null, constraints, 0.25);
    }

    public TrajectorySequenceBuilder lineTo(Vector2d endPosition) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineTo(endPosition, constraints);
        });
    }

    public TrajectorySequenceBuilder lineTo(Vector2d endPosition, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineTo(endPosition, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder lineToConstantHeading(Vector2d endPosition) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineToConstantHeading(endPosition, constraints);
        });
    }

    public TrajectorySequenceBuilder lineToConstantHeading(Vector2d endPosition, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineToConstantHeading(endPosition, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineToLinearHeading(endPose, constraints);
        });
    }

    public TrajectorySequenceBuilder lineToLinearHeading(Pose2d endPose, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineToLinearHeading(endPose, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder lineToSplineHeading(Pose2d endPose) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineToSplineHeading(endPose, constraints);
        });
    }

    public TrajectorySequenceBuilder lineToSplineHeading(Pose2d endPose, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.lineToSplineHeading(endPose, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder strafeTo(Vector2d endPosition) {
        return addPath(() -> {
            currentTrajectoryBuilder.strafeTo(endPosition, constraints);
        });
    }

    public TrajectorySequenceBuilder strafeTo(Vector2d endPosition, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.strafeTo(endPosition, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder forward(double distance) {
        return addPath(() -> {
            currentTrajectoryBuilder.forward(distance, constraints);
        });
    }

    public TrajectorySequenceBuilder forward(double distance, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.forward(distance, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder back(double distance) {
        return addPath(() -> {
            currentTrajectoryBuilder.back(distance, constraints);
        });
    }

    public TrajectorySequenceBuilder back(double distance, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.back(distance, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder strafeLeft(double distance) {
        return addPath(() -> {
            currentTrajectoryBuilder.strafeLeft(distance, constraints);
        });
    }

    public TrajectorySequenceBuilder strafeLeft(double distance, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.strafeLeft(distance, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder strafeRight(double distance) {
        return addPath(() -> {
            currentTrajectoryBuilder.strafeRight(distance, constraints);
        });
    }

    public TrajectorySequenceBuilder strafeRight(double distance, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.strafeRight(distance, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder splineTo(Vector2d endPosition, double endHeading) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineTo(endPosition, endHeading, constraints);
        });
    }

    public TrajectorySequenceBuilder splineTo(Vector2d endPosition, double endHeading, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineTo(endPosition, endHeading, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder splineToConstantHeading(Vector2d endPosition, double endHeading) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, constraints);
        });
    }

    public TrajectorySequenceBuilder splineToConstantHeading(Vector2d endPosition, double endHeading, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineToConstantHeading(endPosition, endHeading, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder splineToLinearHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, constraints);
        });
    }

    public TrajectorySequenceBuilder splineToLinearHeading(Pose2d endPose, double endHeading, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineToLinearHeading(endPose, endHeading, constraintsOverride);
        });
    }

    public TrajectorySequenceBuilder splineToSplineHeading(Pose2d endPose, double endHeading) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, constraints);
        });
    }

    public TrajectorySequenceBuilder splineToSplineHeading(Pose2d endPose, double endHeading, DriveConstraints constraintsOverride) {
        return addPath(() -> {
            currentTrajectoryBuilder.splineToSplineHeading(endPose, endHeading, constraintsOverride);
        });
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

        double durationDifference = builtTraj.duration() - lastDurationTraj;
        double displacementDifference = builtTraj.getPath().length() - lastDisplacementTraj;

        lastPose = builtTraj.end();
        currentDuration += durationDifference;
        currentDisplacement += displacementDifference;

        lastDurationTraj = builtTraj.duration();
        lastDisplacementTraj = builtTraj.getPath().length();

        return this;
    }

    public final TrajectorySequenceBuilder setTangent(double tangent) {
        setAbsoluteTangent = true;
        absoluteTangent = tangent;

        pushPath();

        return this;
    }

    public final TrajectorySequenceBuilder setTangentOffset(double offset) {
        setAbsoluteTangent = false;

        this.tangentOffset = offset;
        this.pushPath();

        return this;
    }

    public final TrajectorySequenceBuilder setReversed(boolean reversed) {
        return reversed ? this.setTangentOffset(Math.toRadians(180.0)) : this.setTangentOffset(0.0);
    }

    public final TrajectorySequenceBuilder setConstraints(DriveConstraints constraints) {
        this.constraints = constraints;

        return this;
    }

    public final TrajectorySequenceBuilder addTemporalMarker(MarkerCallback callback) {
        return this.addTemporalMarker(currentDuration, callback);
    }

    public final TrajectorySequenceBuilder addTemporalMarker(double time, MarkerCallback callback) {
        return this.addTemporalMarker(0.0, time, callback);
    }

    public final TrajectorySequenceBuilder addTemporalMarker(double scale, double offset, MarkerCallback callback) {
        return addDisplacementMarker(new Function1() {
            // Decompiled kotlin

            // $FF: synthetic method
            // $FF: bridge method
            public Object invoke(Object var1) {
                return this.invoke(((Number) var1).doubleValue());
            }

            public final double invoke(double it) {
                return scale * it + offset;
            }
        }, callback);
    }

    public final TrajectorySequenceBuilder addTemporalMarker(Function1<? super Double, Double> time, MarkerCallback callback) {
        this.temporalMarkers.add(new TemporalMarker(time, callback));
        return this;
    }

    public final TrajectorySequenceBuilder addSpatialMarker(Vector2d point, MarkerCallback callback) {
        this.spatialMarkers.add(new SpatialMarker(point, callback));
        return this;
    }

    public final TrajectorySequenceBuilder addDisplacementMarker(MarkerCallback callback) {
        return this.addDisplacementMarker(this.currentDisplacement, callback);
    }

    public final TrajectorySequenceBuilder addDisplacementMarker(double displacement, MarkerCallback callback) {
        return this.addDisplacementMarker(0.0, displacement, callback);
    }

    public TrajectorySequenceBuilder addDisplacementMarker(double scale, double offset, MarkerCallback callback) {
        return addDisplacementMarker(new Function1() {
            // Decompiled kotlin

            // $FF: synthetic method
            // $FF: bridge method
            public Object invoke(Object var1) {
                return this.invoke(((Number) var1).doubleValue());
            }

            public final double invoke(double it) {
                return scale * it + offset;
            }
        }, callback);
    }

    public TrajectorySequenceBuilder addDisplacementMarker(Function1<? super Double, Double> displacement, MarkerCallback callback) {
        displacementMarkers.add(new DisplacementMarker(displacement, callback));

        return this;
    }

    public TrajectorySequenceBuilder turn(double angle) {
        pushPath();

        MotionProfile turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lastPose.getHeading(), 0.0, 0.0, 0.0),
                new MotionState(lastPose.getHeading() + angle, 0.0, 0.0, 0.0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        sequenceSegments.add(new TurnSegment(lastPose, angle, turnProfile, new ArrayList<>()));

        lastPose = lastPose.plus(new Pose2d(0, 0, angle));

        currentDuration += turnProfile.duration();

        return this;
    }

    public TrajectorySequenceBuilder waitSeconds(double seconds) {
        pushPath();
        sequenceSegments.add(new WaitSegment(lastPose, seconds, new ArrayList<>()));

        currentDuration += seconds;
        return this;
    }

    public TrajectorySequenceBuilder addTrajectory(Trajectory trajectory) {
        pushPath();

        sequenceSegments.add(new TrajectorySegment(trajectory));
        return this;
    }

    private void pushPath() {
        if (currentTrajectoryBuilder != null) {
            Trajectory builtTraj = currentTrajectoryBuilder.build();
            sequenceSegments.add(new TrajectorySegment(builtTraj));
        }

        currentTrajectoryBuilder = null;
    }

    private void newPath() {
        if (currentTrajectoryBuilder != null)
            pushPath();

        lastDurationTraj = 0.0;
        lastDisplacementTraj = 0.0;

        double tangent = setAbsoluteTangent ? absoluteTangent : Angle.norm(lastPose.getHeading() + tangentOffset);

        currentTrajectoryBuilder = new TrajectoryBuilder(lastPose, tangent, constraints, resolution);
    }

    public TrajectorySequence build() {
        pushPath();

        List<TrajectoryMarker> globalMarkers = convertMarkersToGlobal(
                sequenceSegments,
                temporalMarkers, displacementMarkers, spatialMarkers
        );

        return projectGlobalMarkersToLocalSegments(globalMarkers, sequenceSegments);
    }

    private List<TrajectoryMarker> convertMarkersToGlobal(
            TrajectorySequence sequenceSegments,
            List<TemporalMarker> temporalMarkers,
            List<DisplacementMarker> displacementMarkers,
            List<SpatialMarker> spatialMarkers
    ) {
        ArrayList<TrajectoryMarker> trajectoryMarkers = new ArrayList<>();

        // Convert temporal markers
        for (TemporalMarker marker : temporalMarkers) {
            trajectoryMarkers.add(
                    new TrajectoryMarker(marker.getTime().invoke(currentDuration), marker.getCallback())
            );
        }

        // Convert displacement markers
        for (DisplacementMarker marker : displacementMarkers) {
            double time =
                    displacementToTime(
                            sequenceSegments,
                            marker.getDisplacement().invoke(
                                    sequenceTotalDisplacement(sequenceSegments)
                            )
                    );

            trajectoryMarkers.add(
                    new TrajectoryMarker(
                            time,
                            marker.getCallback()
                    )
            );
        }

        // Convert spatial markers
        for (SpatialMarker marker : spatialMarkers) {
            trajectoryMarkers.add(
                    new TrajectoryMarker(
                            pointToTime(sequenceSegments, marker.getPoint()),
                            marker.getCallback()
                    )
            );
        }

        return trajectoryMarkers;
    }

    private TrajectorySequence projectGlobalMarkersToLocalSegments(List<TrajectoryMarker> markers, TrajectorySequence sequenceSegments) {
        if (sequenceSegments.isEmpty()) return new TrajectorySequence();

        for (TrajectoryMarker marker : markers) {
            SequenceSegment segment = null;
            int segmentIndex = 0;
            double segmentOffsetTime = 0;

            double currentTime = 0;
            for (int i = 0; i < sequenceSegments.size(); i++) {
                SequenceSegment seg = sequenceSegments.get(i);

                double markerTime = Math.min(marker.getTime(), sequenceSegments.duration());

                if (currentTime + seg.getDuration() >= markerTime) {
                    segment = seg;
                    segmentIndex = i;
                    segmentOffsetTime = markerTime - currentTime;

                    break;
                } else {
                    currentTime += seg.getDuration();
                }
            }

            SequenceSegment newSegment = null;

            if (segment instanceof WaitSegment) {
                List<TrajectoryMarker> newMarkers = segment.getMarkers();
                newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                WaitSegment thisSegment = (WaitSegment) segment;
                newSegment = new WaitSegment(thisSegment.getStartPose(), thisSegment.getDuration(), thisSegment.getMarkers());
            } else if (segment instanceof TurnSegment) {
                List<TrajectoryMarker> newMarkers = segment.getMarkers();
                newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                TurnSegment thisSegment = (TurnSegment) segment;
                newSegment = new TurnSegment(thisSegment.getStartPose(), thisSegment.getTotalRotation(), thisSegment.getMotionProfile(), thisSegment.getMarkers());
            } else if (segment instanceof TrajectorySegment) {
                TrajectorySegment thisSegment = (TrajectorySegment) segment;

                List<TrajectoryMarker> newMarkers = thisSegment.getTrajectory().getMarkers();
                newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                newSegment = new TrajectorySegment(new Trajectory(thisSegment.getTrajectory().getPath(), thisSegment.getTrajectory().getProfile(), newMarkers));
            }

            sequenceSegments.set(segmentIndex, newSegment);
        }

        return sequenceSegments;
    }

    private Double sequenceTotalDisplacement(List<SequenceSegment> sequenceSegments) {
        double total = 0.0;

        for (SequenceSegment segment : sequenceSegments) {
            if (segment instanceof TrajectorySegment) {
                total += ((TrajectorySegment) segment).getTrajectory().getPath().length();
            }
        }

        return total;
    }

    // Taken from Road Runner's TrajectoryGenerator.displacementToTime() since it's private
    // note: this assumes that the profile position is monotonic increasing
    private Double motionProfileDisplacementToTime(MotionProfile profile, double s) {
        double tLo = 0.0;
        double tHi = profile.duration();
        while (!(Math.abs(tLo - tHi) < 1e-6)) {
            double tMid = 0.5 * (tLo + tHi);
            if (profile.get(tMid).getX() > s) {
                tHi = tMid;
            } else {
                tLo = tMid;
            }
        }
        return 0.5 * (tLo + tHi);
    }

    private Double displacementToTime(List<SequenceSegment> sequenceSegments, double s) {
        double currentTime = 0.0;
        double currentDisplacement = 0.0;

        for (SequenceSegment segment : sequenceSegments) {
            if (segment instanceof TrajectorySegment) {
                TrajectorySegment thisSegment = (TrajectorySegment) segment;

                double segmentLength = thisSegment.getTrajectory().getPath().length();

                if (currentDisplacement + segmentLength > s) {
                    double target = s - currentDisplacement;
                    double timeInSegment = motionProfileDisplacementToTime(
                            thisSegment.getTrajectory().getProfile(),
                            target
                    );

                    return currentTime + timeInSegment;
                } else {
                    currentDisplacement += segmentLength;
                    currentTime += thisSegment.getTrajectory().duration();
                }
            } else {
                currentTime += segment.getDuration();
            }
        }

        return 0.0;
    }

    private Double pointToTime(List<SequenceSegment> sequenceSegments, Vector2d point) {
        final class ComparingPoints {
            private final double distanceToPoint;
            private final double totalDisplacement;
            private final double thisPathDisplacement;

            public ComparingPoints(double distanceToPoint, double totalDisplacement, double thisPathDisplacement) {
                this.distanceToPoint = distanceToPoint;
                this.totalDisplacement = totalDisplacement;
                this.thisPathDisplacement = thisPathDisplacement;
            }

            public final double getDistanceToPoint() {
                return this.distanceToPoint;
            }

            public final double getTotalDisplacement() {
                return this.totalDisplacement;
            }

            public final double getThisPathDisplacement() {
                return this.thisPathDisplacement;
            }
        }

        List<ComparingPoints> projectedPoints = new ArrayList<>();

        for (SequenceSegment segment : sequenceSegments) {
            if (segment instanceof TrajectorySegment) {
                TrajectorySegment thisSegment = (TrajectorySegment) segment;

                double displacement = thisSegment.getTrajectory().getPath().project(point, 0.25);
                Vector2d projectedPoint = thisSegment.getTrajectory().getPath().get(displacement).vec();
                double distanceToPoint = point.minus(projectedPoint).norm();

                double totalDisplacement = 0.0;

                for (ComparingPoints comparingPoint : projectedPoints) {
                    totalDisplacement += comparingPoint.totalDisplacement;
                }

                totalDisplacement += displacement;

                projectedPoints.add(new ComparingPoints(distanceToPoint, displacement, totalDisplacement));
            }
        }

        ComparingPoints closestPoint = null;

        for (ComparingPoints comparingPoint : projectedPoints) {
            if (closestPoint == null) {
                closestPoint = comparingPoint;
                continue;
            }

            if (comparingPoint.distanceToPoint < closestPoint.distanceToPoint)
                closestPoint = comparingPoint;
        }

        return displacementToTime(sequenceSegments, closestPoint.thisPathDisplacement);
    }

    // Required for kotlin translation
    // No java equivalent
    private interface AddPathCallback {
        void run();
    }
}
