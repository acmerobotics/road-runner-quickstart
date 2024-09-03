package org.firstinspires.ftc.teamcode.Offseason_Code_Natalia.CV.Opmode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SummerTrainingCourse.CVproject.Pipeline;
import org.firstinspires.ftc.vision.VisionPortal;

    @TeleOp
    public class ihopethisworks extends LinearOpMode {

        //Create Vision Portal for Camera
        VisionPortal visionPortal;

        Pipeline PipelineFromScratch = new Pipeline();

        Pipeline.Location location;


        @Override
        public void runOpMode() throws InterruptedException {

            PipelineFromScratch.color = Pipeline.Color.Blue;

            // Apply Camera Settings and Enable live view
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(1920,1080))
                    .addProcessor(PipelineFromScratch)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();

            visionPortal.setProcessorEnabled(PipelineFromScratch,true);

            while(!isStopRequested() && opModeInInit()){

                telemetry.addData("Location", PipelineFromScratch.getLocation().toString());
                telemetry.update();
            }
            location = PipelineFromScratch.location;

            visionPortal.setProcessorEnabled(PipelineFromScratch,false);

            while(opModeIsActive() && !isStopRequested()){

                if(location == Pipeline.Location.Left){
                    //RUN AUTO LEFT SIDE
                }

                telemetry.update();
            }

        }

    }


