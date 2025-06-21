package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;/*

package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.redRecognizer.pixelLocationRed.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.redRecognizer.pixelLocationRed.MIDDLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.redRecognizer.pixelLocationRed.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.redRecognizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous (name = "redFront auto ")

public class redFront extends LinearOpMode  {

    //THIS IS THE DEFAULT CODE and the DEFAULT POSITION IS BLUE BACK
    private final Pose2d opStartpose = new Pose2d(12, -60, Math.toRadians(90));
    private final Pose2d opEndPose = new Pose2d(50, -60, Math.toRadians(0));

    // line Poses wrote the list this way to make it easier to read.
    private final Pose2d rightLine = new Pose2d(24, -30, Math.toRadians(0));
    private final Pose2d leftLine = new Pose2d(1,-34, Math.toRadians(180));
    private final Pose2d centerLine = new Pose2d(12,-24 , Math.toRadians(90));
    List<Pose2d> listPose = Arrays.asList(leftLine, rightLine, centerLine);

    // these poses are markers for the Left and right back to move to the backdrop in FrontSide Code these will not be here
     private final Pose2d redBackTrussMarker = new Pose2d(12, -36, Math.toRadians(0));

    // these are the drop poses these are the same for all blue opModes only different for Red side
    private final Pose2d redDropL = new Pose2d(50, -28, Math.toRadians(0));
    private final Pose2d redDropC = new Pose2d(50, -36, Math.toRadians(0));
    private final Pose2d redDropR = new Pose2d(50, -44, Math.toRadians(0));

    List<Pose2d> listYellowDrop = Arrays.asList(redDropL, redDropC, redDropR);

    //Sample mecanum drive is the ROBOT class that gives us the ability to create the Robot object in this case name drive and send and revice all the data for Moters, servos ,etc.
    SampleMecanumDrive drive;

    // This is the Microsoft Life Cam 3000
    OpenCvWebcam webcam1 = null;

    redRecognizer ourCam = new redRecognizer();
    redRecognizer.pixelLocationRed linePlace;

    int width = 1280, height = 720;

    private final double travelSpeed = 45.0, travelAccel = 30.0;

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
                webcam1.startStreaming(width, height, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("CameraInitialization", "Camera initialization error: " + errorCode);
            }
        });

        while (!isStarted()) {
            drive.initArm();
            linePlace = ourCam.getPixelLocationRed();
            telemetry.addData("Direction", linePlace);
            telemetry.update();
        }

        waitForStart();
        webcam1.stopStreaming();

        linePlace = ourCam.getPixelLocationRed();


        // because our OP modes are created for each corner
        while(isStarted()) {
            if (linePlace == LEFT) {
                moveToPurple(drive,0);
                drive.setFrontGrip(false);
                LRCmoveToBackDrop (drive, 0);
                drive.setBackGrip(false);
                goPark(drive);
                break;

            }
            else if (linePlace == MIDDLE) {
                moveToPurple(drive,1);
                drive.setFrontGrip(false);
                LRCmoveToBackDrop(drive,1);
                goPark(drive);
                break;
            }
            else if (linePlace == RIGHT) {
                moveToPurple(drive,2);
                drive.setFrontGrip(false);
                LRCmoveToBackDrop (drive, 2);
                goPark(drive);
                break;
            }
        }
    }

    public void moveToPurple (SampleMecanumDrive robot , int LinePlace) {

        robot.setPoseEstimate(opStartpose);
        TrajectorySequence starttoPurple= robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .lineToSplineHeading(redBackTrussMarker,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .lineToSplineHeading(listPose.get(LinePlace),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )

                .build();
        robot.followTrajectorySequence(starttoPurple);
        robot.updatePoseEstimate();


    }
    public void LRCmoveToBackDrop (SampleMecanumDrive robot , int DropPlace) {

        TrajectorySequence LRCgoTOBack= robot.trajectorySequenceBuilder(robot.getPoseEstimate())

                .lineToSplineHeading(listYellowDrop.get(DropPlace),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        robot.followTrajectorySequence(LRCgoTOBack);
        robot.updatePoseEstimate();


    }
    public void goPark (SampleMecanumDrive robot){
        TrajectorySequence  park = robot.trajectorySequenceBuilder(robot.getPoseEstimate())

                .lineToSplineHeading(opEndPose, SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        robot.followTrajectorySequence(park);
        robot.updatePoseEstimate();
    }

}
*/
