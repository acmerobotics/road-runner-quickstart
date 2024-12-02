package org.firstinspires.ftc.teamcode.Auto.Vision;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Teleop.Teleop;

import java.nio.file.Paths;
import java.util.List;

public class Limelight extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();


        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName(); // What was detected
                double xrad = detection.getTargetXDegrees() * 3.1416 / 180; // Where it is (left-right) these are spherical coordinates
                double yrad = detection.getTargetYDegrees() * 3.14156 / 180; // Where it is (up-down)
                telemetry.addData(className, "at (" + xrad + ", " + yrad + ") degrees");
                double height = 1; //height of camera off ground, change value
                double tx = height * Math.sin(yrad) * Math.cos(xrad);//convert xrad and yrad into 2D coordinates
                double ty = height * Math.sin(yrad) * Math.sin(xrad);

            }



        }
    }

    public double getXStrafe (double tx){
        return tx;
    }

    public double getYStrafe (double ty){
        return ty;
    }
}








