package org.firstinspires.ftc.teamcode.Math.AsymmetricProfile;

import org.firstinspires.ftc.teamcode.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Math.Geometry.Rotation2d;

import java.util.ArrayList;

public class DirectTrajectory {
    protected ArrayList<Pose2d> points;
    protected ArrayList<Double> pointTimes;
    public double endTime;

    public DirectTrajectory(ArrayList<Pose2d> points, ArrayList<Double> pointTimes) {
        this.points = points;
        this.pointTimes = pointTimes;

        this.endTime = pointTimes.get(points.size() - 1);
    }

    public Pose2d targetPose(double time) {
        if (time >= endTime) {
            return points.get(points.size() - 1);
        }

        int pointIndex = 0;
        while (pointIndex + 1 < points.size() && pointTimes.get(pointIndex + 1) < time)
            pointIndex++;

        Pose2d latestPose = points.get(pointIndex);
        double latestPoseTime = pointTimes.get(pointIndex);

        Pose2d nextPose = points.get(pointIndex + 1);
        double nextPoseTime = pointTimes.get(pointIndex + 1);

        Pose2d interpolatedPose = new Pose2d(
                 latestPose.getX() + (nextPose.getX() - latestPose.getX()) * (time - latestPoseTime) / (nextPoseTime - latestPoseTime),
                latestPose.getY() + (nextPose.getY() - latestPose.getY()) * (time - latestPoseTime) / (nextPoseTime - latestPoseTime),
                new Rotation2d(latestPose.angleBetween(nextPose))
        );

        return interpolatedPose;
    }

    public Pose2d nextPose(double time) {
        if (time >= endTime)
            return points.get(points.size() - 1);

        int pointIndex = 0;
        while (pointIndex + 1 < points.size() && pointTimes.get(pointIndex + 1) < time)
            pointIndex++;

        Pose2d nextPose = points.get(pointIndex + 1);

        return nextPose;
    }

    public ArrayList<Pose2d> getPoses() {
        return points;
    }
}
