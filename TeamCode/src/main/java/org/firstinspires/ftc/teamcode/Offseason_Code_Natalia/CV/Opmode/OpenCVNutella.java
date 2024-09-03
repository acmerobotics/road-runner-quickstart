package org.firstinspires.ftc.teamcode.Offseason_Code_Natalia.CV.Opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Testing$Tuning.Subsystems$Tele.CV.CV_Pipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;


@TeleOp
    public class OpenCVNutella extends LinearOpMode {
        CV_Pipeline pipeline;
        CV_Pipeline.Location location;
        VisionPortal visionPortal;




        @Override
        public void runOpMode() {

            pipeline  = new CV_Pipeline();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(1920,1080))
                    .addProcessor(pipeline)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();

            visionPortal.setProcessorEnabled(pipeline, true);

            while(!isStopRequested() && opModeInInit()){
                location = pipeline.getLocation();
                telemetry.addData("Prop Location", location.toString());
                telemetry.addData("if you're reading this,at least the telemetry's working", toString());
                telemetry.update();
            }

            //final location
            location = pipeline.getLocation();

            visionPortal.setProcessorEnabled(pipeline, false);
            waitForStart();

            while (opModeIsActive() && !isStopRequested()) {
                telemetry.update();
            }


        }


    }

