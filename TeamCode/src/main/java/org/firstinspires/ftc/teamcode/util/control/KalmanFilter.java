package org.firstinspires.ftc.teamcode.util.control;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class KalmanFilter {
    double kalmanGainX = 1;
    double kalmanGainY = 1;
    double estimateXVariance = 10;
    double estimateYVariance = 10;
    Pose2d estimateState;
    Pose2d calculatedState;
    MecanumDrive drive;
    double lastTime;
    ElapsedTime timer;


    public KalmanFilter(Pose2d startPose){
        estimateState = startPose;
        timer = new ElapsedTime();
        lastTime = 0;
    }
    public void filter(double variance, Pose2d measurement) {
        kalmanGainX = (estimateXVariance)/(estimateXVariance + variance);
        kalmanGainY = (estimateYVariance)/(estimateYVariance + variance);

        estimateXVariance = (1-kalmanGainX)*estimateXVariance; // p n-1,n -> p n,n
        estimateYVariance = (1-kalmanGainY)*estimateYVariance;

        estimateState = new Pose2d(estimateState.getX() + kalmanGainX*(measurement.getX() - estimateState.getX()),
                estimateState.getY() + kalmanGainY*(measurement.getY() - estimateState.getY()),
                new Rotation2d(drive.pose.heading.toDouble()));
        calculatedState = new Pose2d(estimateState.getX(), estimateState.getY(), estimateState.getRotation());

        //predict
        PoseVelocity2d poseVel = drive.updatePoseEstimate();
        Vector2d translatedVel = new Vector2d(poseVel.linearVel.x, poseVel.linearVel.y);
        translatedVel = rotateVector(translatedVel, drive.pose.heading.toDouble());

        double time = timer.seconds() - lastTime;
        estimateXVariance += time*time*Math.abs(translatedVel.x); // p n,n -> p n+1, n
        estimateYVariance += time*time*Math.abs(translatedVel.y);


        estimateState = new Pose2d(estimateState.getX() + translatedVel.x * time,estimateState.getY() + translatedVel.y * time
                ,new Rotation2d(drive.pose.heading.toDouble()));
        lastTime = timer.seconds();


    }

    public Vector2d rotateVector(Vector2d vector,double angle){
        return new Vector2d(Math.cos(angle)*vector.x - Math.sin(angle)*vector.y, Math.sin(angle)*vector.x + Math.cos(angle)*vector.y);
    }
}
