package org.firstinspires.ftc.teamcode.SummerTrainingCourse.OpModes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SummerTrainingCourse.CVproject.Pipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class TestCV extends LinearOpMode {

    //Create Vision Portal for Camera
    VisionPortal visionPortal;

    Pipeline pipeline = new Pipeline();

    Pipeline.Location location;


    @Override
    public void runOpMode() throws InterruptedException {

        pipeline.color = Pipeline.Color.Blue;

        // Apply Camera Settings and Enable live view
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920,1080))
                .addProcessor(pipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(pipeline,true);

        while(!isStopRequested() && opModeInInit()){

            telemetry.addData("Location",pipeline.getLocation().toString());
            telemetry.update();
        }
        location = pipeline.location;

        visionPortal.setProcessorEnabled(pipeline,false);

        while(opModeIsActive() && !isStopRequested()){

            if(location == Pipeline.Location.Left){
                //RUN AUTO LEFT SIDE
            }

            telemetry.update();
        }

    }

}
