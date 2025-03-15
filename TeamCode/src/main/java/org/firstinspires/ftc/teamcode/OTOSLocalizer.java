package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.OTOSKt;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OTOSLocalizer implements Localizer {
    public static class Params {
        public double angularScalar = 1.0;
        public double linearScalar = 1.0;

        // Note: units are in inches and radians
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
    }

    public static Params PARAMS = new Params();

    public final SparkFunOTOS otos;
    private Pose2d currentPose;

    public OTOSLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        // TODO: make sure your config has an OTOS device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        currentPose = initialPose;
        otos.setPosition(OTOSKt.toOTOSPose(currentPose));
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.calibrateImu();
        otos.setLinearScalar(PARAMS.linearScalar);
        otos.setAngularScalar(PARAMS.angularScalar);
        otos.setOffset(PARAMS.offset);
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
        otos.setPosition(OTOSKt.toOTOSPose(currentPose));
    }

    @Override
    public PoseVelocity2d update() {
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);

        currentPose = OTOSKt.toRRPose(otosPose);
        Vector2d fieldVel = new Vector2d(otosVel.x, otosVel.y);
        Vector2d robotVel = Rotation2d.exp(otosPose.h).inverse().times(fieldVel);
        return new PoseVelocity2d(robotVel, otosVel.h);
    }
}
