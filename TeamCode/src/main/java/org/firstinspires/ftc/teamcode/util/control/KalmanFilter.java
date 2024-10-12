package org.firstinspires.ftc.teamcode.util.control;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;

public class KalmanFilter {
    double kalmanGainX = 1;
    double kalmanGainY = 1;
    double estimateXVariance = 2;
    double estimateYVariance = 2;
    Pose2d estimateState;
    Pose2d calculatedState;
    GoBildaPinpoint pinpoint;
    Limelight3A limelight;
    ElapsedTime timer;
    Vector2d translatedVel;


    double ODO_VARIANCE = 10;
    double X_MULT_VARIANCE = 1; //for apriltags
    double Y_MULT_VARIANCE = 1;
    double DEG_MULT_VARIANCE = 1;
    double TAG_VARIANCE = 2;

    public KalmanFilter(Pose2d startPose, GoBildaPinpoint pinpoint, Limelight3A limelight){
        estimateState = startPose;
        timer = new ElapsedTime();
        this.pinpoint = pinpoint;
        this.limelight = limelight;
    }
    public void filterKalman(double variance, Pose2d measurement, Telemetry telemetry) {
        kalmanGainX = (estimateXVariance)/(estimateXVariance + variance);
        kalmanGainY = (estimateYVariance)/(estimateYVariance + variance);
        telemetry.addData("kalmanGainX", kalmanGainX);
        telemetry.addData("kalmanGainY", kalmanGainY);

        estimateXVariance = (1-kalmanGainX)*estimateXVariance; // p n-1,n -> p n,n
        estimateYVariance = (1-kalmanGainY)*estimateYVariance;
        telemetry.addData("estimateX", estimateXVariance);
        telemetry.addData("estimateY", estimateYVariance);
        telemetry.addData("measurement", measurement);
        telemetry.addData("estimateState", estimateState);
        estimateState = new Pose2d(estimateState.getX() + kalmanGainX*(measurement.getX() - estimateState.getX()),
                estimateState.getY() + kalmanGainY*(measurement.getY() - estimateState.getY()),
                new Rotation2d(pinpoint.getHeading()));
        calculatedState = new Pose2d(estimateState.getX(), estimateState.getY(), estimateState.getRotation());

        telemetry.addData("calculatedState", calculatedState);


        //predict
        translatedVel = new Vector2d(pinpoint.getVelX()*0.03937, pinpoint.getVelY()*0.03937);
        //translatedVel = rotateVector(translatedVel, pinpoint.getHeading());

        double time = timer.time();
        timer.reset();
        telemetry.addData("translatedVel", translatedVel);
        estimateXVariance += time*time*Math.abs(translatedVel.x); // p n,n -> p n+1, n
        estimateYVariance += time*time*Math.abs(translatedVel.y);


        estimateState = new Pose2d(estimateState.getX() + translatedVel.x * time
                ,estimateState.getY() + translatedVel.y * time
                ,new Rotation2d(pinpoint.getHeading()));
        telemetry.update();
    }

    public void aprilTagKalman(Telemetry telemetry){
        filterKalman(ODO_VARIANCE, new Pose2d(pinpoint.getPosX(), pinpoint.getPosY(), new Rotation2d(0)), telemetry);
        Pose2d atagPose = new Pose2d(limelight.getLatestResult().getBotpose().getPosition().x*39.3701007874, limelight.getLatestResult().getBotpose().getPosition().y*39.3701007874, new Rotation2d(pinpoint.getHeading()));

        double xVar = Math.abs(translatedVel.x * X_MULT_VARIANCE);
        double yVar = Math.abs(translatedVel.y * Y_MULT_VARIANCE);
        double degVar = Math.abs (limelight.getLatestResult().getTa() * DEG_MULT_VARIANCE);
        double totalTagVar = xVar + yVar + degVar + Math.abs (Math.abs (Math.toDegrees (limelight.getLatestResult().getTa())) - 180) + TAG_VARIANCE;
        if (limelight.getLatestResult().isValid()) {filterKalman (totalTagVar, atagPose, telemetry);}
    }

    public void odoKalman(Telemetry telemetry){
        filterKalman(ODO_VARIANCE, new Pose2d(pinpoint.getPosX()*0.03937, pinpoint.getPosY()*0.03937, new Rotation2d(0)),telemetry);
    }
    public Pose2d getCalculatedState(){
        return this.calculatedState;
    }

    public Vector2d rotateVector(Vector2d vector,double angle){
        return new Vector2d(Math.cos(angle)*vector.x - Math.sin(angle)*vector.y, Math.sin(angle)*vector.x + Math.cos(angle)*vector.y);
    }
}
