package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.visionIntoTheDeep.itdCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "visionTest", group = "learn")
@Config
public class visionTest extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    itdCameraPipeline itdCam = new itdCameraPipeline();

    itdCameraPipeline.sampleAngleStage anglePlace;
    itdCameraPipeline.sampleHeightStage heightPlace;

    int width = 1280, height = 720;

    @Override
    public void runOpMode() throws InterruptedException {

        // Set up the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // The view ID is needed for displaying the camera feed in the FTC app
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Initialize the webcam
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Set the camera's pipeline to the one you've created
        webcam1.setPipeline(itdCam);

        // Open the camera asynchronously to avoid blocking the thread
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming in MJPEG format with a specified resolution
                webcam1.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened: " + errorCode);
                telemetry.update();
            }
        });

        // Wait for the start command from the driver station
        waitForStart();

        // Main opMode loop
        while (opModeIsActive()) {
            // Add any logic or telemetry here to monitor pipeline results
            telemetry.addData("Frame Count", webcam1.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam1.getFps()));
            telemetry.update();
        }

        // Stop the camera when the opMode is finished
        webcam1.stopStreaming();
    }
}