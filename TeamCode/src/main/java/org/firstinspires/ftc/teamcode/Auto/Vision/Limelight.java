package org.firstinspires.ftc.teamcode.Auto.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Teleop.Teleop;

import java.nio.file.Paths;
import java.util.List;

@TeleOp
public class Limelight extends LinearOpMode {
    public Limelight3A limelight;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry tele = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
//            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
//            for (LLResultTypes.DetectorResult detection : detections) {
//                String className = detection.getClassName(); // What was detected
//                double xrad = detection.getTargetXDegrees() * 3.1416 / 180; // Where it is (left-right) these are spherical coordinates
//                double yrad = detection.getTargetYDegrees() * 3.14156 / 180; // Where it is (up-down)
//
//                double height = 1; //height of camera off ground, change value
//                double tx = height * Math.sin(yrad) * Math.cos(xrad);//convert xrad and yrad into 2D coordinates
//                double ty = height * Math.sin(yrad) * Math.sin(xrad);
//                telemetry.addData(className, "at (" + xrad + ", " + yrad + ") degrees");
//                telemetry.addData("a", tx + "tx" + ty + "ty");
//
//            }
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.addData("a", "a");
            telemetry.update();



        }
    }

//    public double getXStrafe (double tx){
//        return tx;
//    }
//
//    public double getYStrafe (double ty){
//        return ty;
//    }
}








