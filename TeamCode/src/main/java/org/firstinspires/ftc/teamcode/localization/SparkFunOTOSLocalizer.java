package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class SparkFunOTOSLocalizer implements Localizer {
    SparkFunOTOS otos;
    Pose2d pose;

    public SparkFunOTOSLocalizer(SparkFunOTOS otos, Pose2d pose) {
        this.otos = otos;
        this.pose = pose;
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
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        SparkFunOTOS.Pose2D[] otosResults = getPoseVelAcc();

        SparkFunOTOS.Pose2D otosPose = otosResults[0];
        SparkFunOTOS.Pose2D otosVel = otosResults[1];

        pose = otosPoseToRR(otosPose);

        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y), otosVel.h);
    }

    public static SparkFunOTOS.Pose2D rrPoseToOtos(Pose2d pose)  {
        return new SparkFunOTOS.Pose2D(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    public static Pose2d otosPoseToRR(SparkFunOTOS.Pose2D pose)  {
        return new Pose2d(pose.x, pose.y, pose.h);
    }
}
