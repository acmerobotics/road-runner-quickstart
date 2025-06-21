package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;/*package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.MIDDLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="ryanTriplett", group = "practice")
@Config
public class ryanTriplett extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
    private final Pose2d endPose = new Pose2d(0, 15, Math.toRadians(0));

    private final Pose2d endPose2 = new Pose2d(15, 0, Math.toRadians(0));
    private final Pose2d endPose3 = new Pose2d(15, 0, Math.toRadians(0));

    private final double travelSpeed = 45.0, travelAccel = 30.0;
    private final int width = 1280, height = 720;

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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        // Set the camera's pipeline
        webcam1.setPipeline(ourCam);

        // Open the camera
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("CameraInitialization", "Camera initialization error: " + errorCode);
            }
        });

        while (!isStarted()) {
            drive.initArm();
            linePlace = ourCam.getDirection();
            telemetry.addData("Direction", linePlace);
            telemetry.update();
        }

        waitForStart();
        webcam1.stopStreaming();

        linePlace = ourCam.getDirection();

        TrajectorySequence leftProp = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(endPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                ).build();

        TrajectorySequence middleProp = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(endPose2,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                ).build();

        TrajectorySequence rightProp = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(endPose3,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                ).build();

        while(isStarted()) {
            if (linePlace == LEFT) {
                drive.followTrajectorySequence(leftProp);
            }
            else if (linePlace == MIDDLE) {
                drive.followTrajectorySequence(middleProp);
            }
            else if (linePlace == RIGHT) {
                drive.followTrajectorySequence(rightProp);
            }
        }
    }
}*/