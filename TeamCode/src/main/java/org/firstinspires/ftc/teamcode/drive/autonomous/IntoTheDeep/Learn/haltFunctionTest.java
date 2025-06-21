package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionIntoTheDeep.itdBlobPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Config
@Autonomous(name="haltFunctionTest", group="learn")

public class haltFunctionTest extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    itdBlobPipeline itdCam = new itdBlobPipeline();

    int width = 1280, height = 720;

    @Override
    public void runOpMode() {
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

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-35, -50, Math.toRadians(90)));

    // Trajectory actions are actions (will go more into these later) that create a trajectory
    // A trajectory is a path that the robot can follow.
    Action trajectoryAction1; // Defining a variable called trajectoryAction

    // Now we get into actually building trajectories
    trajectoryAction1 = drive.actionBuilder(drive.localizer.getPose()) // drive.pose is the starting position you gave the robot
            .waitSeconds(0.1)
            .build();

    while (!isStarted()) {
        if (itdCam.cameraBlocked) {
            telemetry.addLine("Camera Blocked");
            Actions.runBlocking(
                    trajectoryAction1
            );
        }
    };
}}
