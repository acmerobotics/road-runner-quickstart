package org.firstinspires.ftc.teamcode.Purepursuit;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Math.Geometry.Rotation2d;

/**
 * point in our pure pursuit curve that we will actively move toward
 */
public class CurvePoint {

    public double x;
    public double y;
    public double moveSpeed;
    public double followDistance;
    protected boolean isObstacle = false;
    public boolean visited = false;

    public final double DEFAULT_FOLLOW_DIST = CurveCalculator.FAST_FOLLOW_DIST_IN;

    public CurvePoint(double x, double y, double moveSpeed, double followDistance) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.followDistance = followDistance;

    }


    public CurvePoint(double x, double y, double moveSpeed) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.followDistance = DEFAULT_FOLLOW_DIST;
    }

    public CurvePoint(double x, double y) {
        this.x = x;
        this.y = y;
        this.moveSpeed = 1;
        this.followDistance = DEFAULT_FOLLOW_DIST;
    }

    public CurvePoint(CurvePoint thisPoint) {
        this.x = thisPoint.x;
        this.y = thisPoint.y;
        this.moveSpeed = thisPoint.moveSpeed;
        this.followDistance = thisPoint.followDistance;
        this.setObstacle(thisPoint.isObstacle());
        this.visited = thisPoint.visited;
    }


    public void setPoint(Point intersection) {
        this.x = intersection.x;
        this.y = intersection.y;
    }


//    @Override
//    public boolean equals(CurvePoint other) {
//        return this.x == other.x && this.y == other.y;
//    }


    @Override
    public boolean equals(@Nullable Object obj) {

        if (obj == this) return true;
        if (!(obj instanceof CurvePoint)) return false;
        CurvePoint that = (CurvePoint) obj;

        return this.x == that.x && this.y == that.y;
    }

    public boolean equals(Point other) {
        return this.x == other.x && this.y == other.y;
    }

    public double distanceTo(CurvePoint other) {
        Pose2d thisPose = new Pose2d(this.x,this.y,new Rotation2d(0));
        Pose2d otherPose = new Pose2d(other.x,other.y,new Rotation2d(0));
        return thisPose.distanceBetween(otherPose);
    }

    public void setObstacle(boolean obstacle) {
        isObstacle = obstacle;
    }

    public boolean isObstacle() {
        return isObstacle;
    }

    @NonNull
    @Override
    public String toString() {
        return "x: " + this.x + " y: " + this.y;
    }


    @Override
    public int hashCode() {
        return (int)x + (int)Math.pow(y,2);
    }
}
