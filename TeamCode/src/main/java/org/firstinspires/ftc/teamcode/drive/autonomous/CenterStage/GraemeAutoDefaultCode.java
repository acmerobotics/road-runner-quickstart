package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.Recognizer.pixelLocation.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.Recognizer.pixelLocation.MIDDLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.Recognizer.pixelLocation.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.Recognizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous (name = "THIS IS BLUE BACK ")

public class GraemeAutoDefaultCode extends LinearOpMode  {

    //THIS IS THE DEFAULT CODE and the DEFAULT POSITION IS BLUE BACK
    //I HAVE NOT WORKED ON HEADINGS
    private final Pose2d opStartpose = new Pose2d(-36, 60, Math.toRadians(-90));
    private final Pose2d opEndPose = new Pose2d(48, 36, Math.toRadians(0));


    // line Poses wrote the list this way to make it easier to read.
    private final Pose2d rightLine = new Pose2d(-48, 36, Math.toRadians(-90));
    private final Pose2d leftLine = new Pose2d(-24, 36, Math.toRadians(0));
    private final Pose2d centerLine = new Pose2d(-36, 24.5, Math.toRadians(-90));
    List<Pose2d> listPose = Arrays.asList(leftLine, centerLine,rightLine);



    // these poses are markers for the Left and right back to move to the backdrop in FrontSide Code these will not be here
    private final Pose2d blueBackTruss = new Pose2d(-20, 36, Math.toRadians(0));
    private final Pose2d blueBackLineMoveMarker = new Pose2d(-36, 41, Math.toRadians(-90));
    private final Pose2d blueFrontTruss = new Pose2d(24, 36, Math.toRadians(0));


    //these are markers for the Center back move2backDrop in FrontSide Cose these will not be here
    private final Pose2d blueFrontGate = new Pose2d(24, 12, Math.toRadians(0));
    private final Pose2d blueBackGate = new Pose2d(-34, 11.5, Math.toRadians(-15));


    // these are the drop poses these are the same for all blue opModes only different for Red side
    private final Pose2d blueDropL = new Pose2d(50, 42, Math.toRadians(0));
    private final Pose2d blueDropC = new Pose2d(50, 36, Math.toRadians(0));
    private final Pose2d blueDropR = new Pose2d(50, 30, Math.toRadians(0));

    List<Pose2d> listYellowDrop = Arrays.asList(blueDropL, blueDropC, blueDropR);

    //Sameple mecnum drive is the ROBOT class that gives us the ablity to create the Robot object in this case name drive and send and revice all the data for Moters, servos ,etc.
    SampleMecanumDrive drive;

    // This is the Microsoft Life Cam 3000
    OpenCvWebcam webcam1 = null;

    Recognizer ourCam = new Recognizer();
    Recognizer.pixelLocation linePlace;

    int width =1280, height = 720;

    private final double travelSpeed = 45.0, travelAccel = 30.0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        // Set up the webcam
    */
/*    WebcamName adjustCameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
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
        });*//*

        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class,"Webcam 1");

        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        webcam1.setPipeline(ourCam);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted()) {
            drive.initArm();
            linePlace = ourCam.getPixelLocation();
            telemetry.addData("Direction", linePlace);
            telemetry.update();
        }

        waitForStart();
        webcam1.stopStreaming();

        linePlace = ourCam.getPixelLocation();


        // because our OP modes are created for each corner
        while(isStarted()) {
            if (linePlace == LEFT) {
                moveToPurple(drive,0);
              //  drive.setFrontGrip(false);
                LRmoveToBackDrop (drive, 0);
              //  drive.setBackGrip(false);
                goPark(drive);
                break;

            }
            else if (linePlace == MIDDLE) {
                moveToPurple(drive,1);
            //    drive.setFrontGrip(false);
                CmoveToBackDrop(drive,1);
                goPark(drive);
                break;
            }
            else if (linePlace == RIGHT) {
                moveToPurple(drive,2);
              //  drive.setFrontGrip(false);
                LRmoveToBackDrop (drive, 2);
                goPark(drive);
                break;
            }
        }
    }

    public void moveToPurple (SampleMecanumDrive robot , int LinePlace) {

        robot.setPoseEstimate(opStartpose);
        TrajectorySequence  starttoPurple= robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .lineToSplineHeading(blueBackLineMoveMarker,
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
    public void LRmoveToBackDrop (SampleMecanumDrive robot , int DropPlace) {

        TrajectorySequence  LRgoTOBack= robot.trajectorySequenceBuilder(robot.getPoseEstimate())

                .lineToSplineHeading(blueBackTruss,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )

                .lineToSplineHeading(blueFrontTruss,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel))

                .lineToSplineHeading(listYellowDrop.get(DropPlace),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )

                .build();
        robot.followTrajectorySequence(LRgoTOBack);
        robot.updatePoseEstimate();


    }
    public void CmoveToBackDrop (SampleMecanumDrive robot , int DropPlace) {

        TrajectorySequence  CgoToBack= robot.trajectorySequenceBuilder(robot.getPoseEstimate())

                .lineToSplineHeading(blueBackGate,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )

                .lineToSplineHeading(blueFrontGate,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel))

                .lineToSplineHeading(listYellowDrop.get(DropPlace),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )

                .build();
        robot.followTrajectorySequence(CgoToBack);
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
