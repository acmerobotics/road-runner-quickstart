package org.firstinspires.ftc.teamcode.Offseason_Code_Natalia.CV.Opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Offseason_Code_Natalia.blahblah.PipelineFromScratch;  // Updated import
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class Opmode2 extends LinearOpMode {
    PipelineFromScratch pipeline;  // Changed from CV_Pipeline to PipelineFromScratch, references PipelinefromScratch
    VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        pipeline = new PipelineFromScratch();  // Instantiate PipelineFromScratch

        visionPortal = new VisionPortal.Builder()
                //setting up camera and vision portal and things
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920,1080))
                .addProcessor(pipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();





        visionPortal.setProcessorEnabled(pipeline, true);
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 30);


        while (!isStopRequested() && opModeInInit()) {
            // Assuming getLocation() is a method you need to implement in PipelineFromScratch
            telemetry.addData("if you're reading this, at least the telemetry's working", toString());
            telemetry.update();
        }

        visionPortal.setProcessorEnabled(pipeline, false);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
        visionPortal.stopStreaming();


    }
}
