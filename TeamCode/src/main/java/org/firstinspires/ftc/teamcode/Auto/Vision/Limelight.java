package org.firstinspires.ftc.teamcode.Auto.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight extends LinearOpMode {
    Limelight3A limelight;



    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);//pipiline number in limelight menu

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);//pipiline number
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        }
        else {
            telemetry.addData("Limelight", "No Targets");
        }



        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        for (LLResultTypes.DetectorResult detection : detections) {
            String className = detection.getClassName(); // What was detected
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
        }

        waitForStart();
         //Starts polling for data.

                limelight.start();
        while (opModeIsActive()) {
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
    }








}







