package org.firstinspires.ftc.teamcode.subsystems.relocalization;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

public class Relocalization {

    public CVMaster vision;
    public SparkFunOTOS otos;
    public GoBildaPinpoint pinpoint;

    public Relocalization(CVMaster cv, GoBildaPinpoint pp, SparkFunOTOS o) {
        vision = cv;
        pinpoint = pp;
        otos = o;
    }

    public Pose2d relocalizeMT2() {
        return vision.mt2Relocalize(pinpoint.getHeading());
    }

    public Pose2d relocalizeMT1() {
        return vision.mt1Relocalize();
    }

    public Pose2d relocalizeOTOS() {
        SparkFunOTOS.Pose2D otosPos = otos.getPosition();
        return new Pose2d(otosPos.x, otosPos.y, otosPos.h);
    }
}
