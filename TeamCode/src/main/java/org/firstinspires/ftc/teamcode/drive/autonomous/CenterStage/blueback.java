package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;


import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.MIDDLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue Back auto")
@Config
public class blueback extends LinearOpMode {

    private final Pose2d blueBackStart = new Pose2d(-36, 60, Math.toRadians(-90));
    private final Pose2d blueBackEnd = new Pose2d(48, 36, Math.toRadians(0));


    // drive forward 2 feet
    private final Pose2d centerOfLines = new Pose2d(-36, 36, Math.toRadians(-90));


    // the poses for the lines on the ground


    private final Pose2d rightLine = new Pose2d(-48, 32, Math.toRadians(0));
    private final Pose2d leftLine = new Pose2d(-24, 32, Math.toRadians(0));
    private final Pose2d centerLine = new Pose2d(-36, 24.5, Math.toRadians(-90));

    // truss
    private final Pose2d blueBackTruss = new Pose2d(-24, 36, Math.toRadians(0));
    private final Pose2d blueFrontGate = new Pose2d(24, 8, Math.toRadians(0));
    private final Pose2d blueBackGate = new Pose2d(-24, 8, Math.toRadians(0));

    private int _pose;

    SampleMecanumDrive drive;
    // This is the Microsoft Life Cam 3000
    OpenCvWebcam webcam1 = null;

    blueCameraPipeline ourCam = new blueCameraPipeline();
    blueCameraPipeline.MovementDirection linePlace;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        // Set up the webcam
        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // adjustCamera was the deviceName in last years code
        // We may need to change the name adjustCamera to Webcam1
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        // Set the camera's pipeline
        webcam1.setPipeline(ourCam);

        // Open the camera
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        while (!isStarted()) {
            linePlace=ourCam.getDirection();
            telemetry.addData("Parking Zone", linePlace);
            telemetry.update();
        }

            // ask gram is second trajectory sequence needs to be the where the last trajectory sequence ended
            if (linePlace == LEFT) {
                drive.setPoseEstimate(blueBackStart);
                TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(blueBackStart)
                        .lineToSplineHeading(leftLine)
                        .build();
                drive.followTrajectorySequence(placePurpleBack);
                drive.setFrontGrip(false);
                sleep(1000);
                TrajectorySequence gotoback = drive.trajectorySequenceBuilder(leftLine)
                        .lineToSplineHeading(blueBackTruss)
                        .lineToSplineHeading(blueBackEnd)
                        .build();
                drive.followTrajectorySequence(gotoback);

            } else if (linePlace == MIDDLE) {
                drive.setPoseEstimate(blueBackStart);
                TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(blueBackStart)
                        .lineToSplineHeading(centerLine)
                        .build();
                drive.followTrajectorySequence(placePurpleBack);
                drive.setFrontGrip(false);
                sleep(1000);
                TrajectorySequence gotoback = drive.trajectorySequenceBuilder(centerLine)
                        .lineToSplineHeading(blueBackGate)
                        .lineToSplineHeading(blueFrontGate)
                        .lineToSplineHeading(blueBackEnd)
                        .build();
                drive.followTrajectorySequence(gotoback);
            } else if (linePlace == RIGHT) {
                drive.setPoseEstimate(blueBackStart);
                TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(blueBackStart)
                        .lineToSplineHeading(rightLine)
                        .build();
                drive.followTrajectorySequence(placePurpleBack);
                drive.setFrontGrip(false);
                sleep(1000);
                TrajectorySequence gotoback = drive.trajectorySequenceBuilder(rightLine)
                        .lineToSplineHeading(blueBackTruss)
                        .lineToSplineHeading(blueBackEnd)
                        .build();
                drive.followTrajectorySequence(gotoback);

            }

        }
    }*/


