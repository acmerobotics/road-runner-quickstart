package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SparkFunOTOSLocalizer implements Localizer {
    SparkFunOTOS otos;
    Pose2d lastPose;

    public SparkFunOTOSLocalizer(SparkFunOTOS otos) {
        this.otos = otos;
    }

    public SparkFunOTOS.Pose2D[] getPoseVelAcc() {
        SparkFunOTOS.Pose2D[] poseVelAcc = new SparkFunOTOS.Pose2D[3];
        poseVelAcc[0] = new SparkFunOTOS.Pose2D();
        poseVelAcc[1] = new SparkFunOTOS.Pose2D();
        poseVelAcc[2] = new SparkFunOTOS.Pose2D();

        otos.getPosVelAcc(poseVelAcc[0], poseVelAcc[1], poseVelAcc[2]);

        return poseVelAcc;
    }

    @Override
    public Pose2d updatePositionEstimate(Pose2d pose) {
        //weird cursed behavior
        if (pose != lastPose) {
            otos.setPosition(rrPoseToOtos(pose));
        }

        SparkFunOTOS.Pose2D otosPose = getPoseVelAcc()[0];

        lastPose = otosPoseToRR(otosPose);
        return lastPose;
    }

    @Override
    public PoseVelocity2d updateVelocityEstimate() {
        SparkFunOTOS.Pose2D otosPose = getPoseVelAcc()[0];
        return new PoseVelocity2d(new Vector2d(otosPose.x, otosPose.y), otosPose.h);
    }

    public static SparkFunOTOS.Pose2D rrPoseToOtos(Pose2d pose)  {
        return new SparkFunOTOS.Pose2D(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    public static Pose2d otosPoseToRR(SparkFunOTOS.Pose2D pose)  {
        return new Pose2d(pose.x, pose.y, pose.h);
    }
}
