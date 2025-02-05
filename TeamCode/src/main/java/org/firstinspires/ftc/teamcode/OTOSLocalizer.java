package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class OTOSLocalizer implements Localizer {
    public static class Params {
        public double angularScalar = 0.0;
        public double linearScalar = 0.0;

        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    }

    public static Pose2d convertPose(SparkFunOTOS.Pose2D pose) {
        return new Pose2d(pose.x, pose.y, pose.h);
    }

    public static SparkFunOTOS.Pose2D convertPose(Pose2d pose) {
        return new SparkFunOTOS.Pose2D(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    public static Params PARAMS = new Params();

    private final SparkFunOTOS otos;
    private Pose2d currentPose;

    public OTOSLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        currentPose = initialPose;
        otos.setPosition(convertPose(currentPose));

        otos.calibrateImu();
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.setOffset(PARAMS.offset);
    }

    public SparkFunOTOS getOTOS() {
        return otos;
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
        otos.setPosition(convertPose(currentPose));
    }

    @Override
    public PoseVelocity2d update() {
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);

        currentPose = convertPose(otosPose);
        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y), otosVel.h);
    }
}
