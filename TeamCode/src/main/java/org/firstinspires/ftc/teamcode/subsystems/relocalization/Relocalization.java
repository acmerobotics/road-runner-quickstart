package org.firstinspires.ftc.teamcode.subsystems.relocalization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

public class Relocalization {

    public CVMaster vision;
    public SparkFunOTOS otos;
    private HardwareMap hardwareMap;

    public Relocalization(CVMaster cv, SparkFunOTOS o, HardwareMap map) {
        vision = cv;
        otos = o;
        hardwareMap = map;
    }

    public Pose2d relocalizeMT2() {
        return vision.mt2Relocalize(otos.getPosition().h);
    }

    public Pose2d relocalizeMT1() {
        return vision.mt1Relocalize();
    }

    public Pose2d relocalizeOTOS() {
        SparkFunOTOS.Pose2D otosPos = otos.getPosition();
        return new Pose2d(otosPos.x, otosPos.y, new Rotation2d(otosPos.h));
    }
}
