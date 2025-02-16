package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {
    private Limelight3A limelight;
    public LLResult result;
    public Pose3D botPose;

    public Limelight(HardwareMap hwmap){
        limelight = hwmap.get(Limelight3A.class, "limelight");
        limelight.start();
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            botPose = result.getBotpose();
        }
    }

    public void updateLimelight(){
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            botPose = result.getBotpose();
        }
    }

    public static double metersToInches(double meters){
        return meters * 39.3701;
    }

    public static double inchesToMeters(double inches){
        return inches / 39.3701;
    }

    public static double distanceBetweenPose(Pose2d p1, Pose2d p2) {
        return Math.sqrt(
                Math.pow(p2.position.x - p1.position.x, 2) + Math.pow(p2.position.y - p1.position.y, 2)
        );
    }

    public static double headingDifferencePose(Pose2d p1, Pose2d p2) {
        return Math.abs(Math.toDegrees(p2.heading.toDouble()) - Math.toDegrees(p1.heading.toDouble()));
    }
}
