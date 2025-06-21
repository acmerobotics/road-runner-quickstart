package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay;/*package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay.parkingZoneFinder;
import org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay.poleFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Disabled
@Autonomous(name = "Red Right Score w/ Camera")
public class redRightScoreCam extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(36, -63, Math.toRadians(90));
    private final Pose2d interPose = new Pose2d(36, -24, Math.toRadians(180));
    private final Pose2d scorePose = new Pose2d(38, -10, Math.toRadians(133));
    private final Pose2d cyclePose = new Pose2d(30, -10, Math.toRadians(133));
    private final Pose2d stackPose = new Pose2d(40, -10, Math.toRadians(5));

    private final double travelSpeed = 45.0, travelAccel = 30.0;
    private final double adjustmentSpeed = 1.5, adjustmentAccel = 1.5;
    private final double angVel = Math.toRadians(120), adjustAngVel = Math.toRadians(20);

    private Pose2d[] parkingSpots = {new Pose2d(12, -12, Math.toRadians(180)), new Pose2d(36, -12, Math.toRadians(180)), new Pose2d(60, -12, Math.toRadians(180))};

    private final int width = 1280, height = 720, slices = 64;

    SampleMecanumDrive drive;
    OpenCvWebcam adjustCamera = null;
    // poleFinder poleFinderPipeline = new poleFinder(width, height, slices);
    poleFinder poleFinderPipeline = new poleFinder();
    poleFinder.poleLocation moveDir;
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize arm
        drive.initArm();

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(scorePose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();



        // Set up the webcam
        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class, "adjustCamera");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        adjustCamera = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        // Set the camera's pipeline
        adjustCamera.setPipeline(parkingZonePipeline);

        // Open the camera
        adjustCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                adjustCamera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted()) {
            zone = parkingZonePipeline.getParkingZone();
            telemetry.addData("Parking Zone", zone);
            telemetry.update();
        }

        adjustCamera.setPipeline(poleFinderPipeline);

        //waitForStart();

        // Close the grip and move the slide up a small amount
        drive.setGrip(true);
        sleep(250);
        drive.setHeight(200);
        drive.setExtension(50);
        drive.setSlideVelocity(4000, drive.slideLeft, drive.slideRight);

        // The sleep is necessary to wait for certain arm actions to finish
        sleep(250);

        // Increase the height of the slide and increase its velocity
        drive.setHeight(3200);
        drive.setSlideVelocity(4000, drive.slideLeft, drive.slideRight);

        drive.followTrajectorySequence(goToStack);

        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location

        //  drive.followTrajectorySequence(goToStack);
        // Update roadrunner's idea of where the robot is after we ran the trajectory
        drive.updatePoseEstimate();
        // Adjust our angle so that we are lined up with the pole
        sleep(250); // Wait for wiggles to stop just in case
        adjustAngle(drive);

        // Increase the slide height to high junction height and increase its velocity //TODO:
        drive.setSlideVelocity(4000, drive.slideLeft, drive.slideRight);
        drive.setHeight(4200);
        // Wait until the slides are high enough that we won't hit the pole when extending
        sleep(500);
        // Extend the horizontal slide above the pole
        drive.setSlideVelocity(4000, drive.slideTop);
        drive.setExtension(960);

        drive.setExtension(825);

        // Wait for arm to be in position
        sleep(1500);

        // Open grip to drop cone
        drive.setGrip(false);

        // Wait for grip to fully open and cone to drop
        sleep(500);

        for (int i = 5; i > 2; i--) {
            toStack(drive, i);
            scoreCone(drive);
        }

        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }
    }

    private void adjustAngle(SampleMecanumDrive _drive) {
        double degrees = 3.0;
        int tries = 1;
        poleFinder.poleLocation prev = poleFinder.poleLocation.ALIGNED;

        while (moveDir != poleFinder.poleLocation.ALIGNED && tries > 0) {
            moveDir = poleFinderPipeline.getLocation();
            if (moveDir == poleFinder.poleLocation.LEFT) {
                if (prev == moveDir) {
                    tries -= 3;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(degrees), Math.toRadians(30.0), Math.toRadians(10.0))
                        .build());
                prev = poleFinder.poleLocation.LEFT;
            } else if (moveDir == poleFinder.poleLocation.RIGHT) {
                if (prev == moveDir) {
                    tries -= 1;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(-degrees), Math.toRadians(30.0), Math.toRadians(10.0))
                        .build());
                prev = poleFinder.poleLocation.RIGHT;
            }
            if (isStopRequested()) {
                break;
            }
        }
    }

    private void toStack(SampleMecanumDrive _drive, int stackHeight ) {
        // stackHeight is given as height of stack in cones
        //step one
        _drive.setExtension(240);
        sleep(1000);

        _drive.setSlideVelocity(4000, _drive.slideTop);
        _drive.setHeight(50 + (stackHeight * 140));
        _drive.setSlideVelocity(4000, drive.slideTop);


        _drive.updatePoseEstimate();
        _drive.followTrajectorySequence
                (_drive.trajectorySequenceBuilder(
                                scorePose)
                        .lineToSplineHeading(stackPose,
                                SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(120), DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(20)
                        )
                        .build()
                );

        _drive.setExtension(2000);
        _drive.setSlideVelocity(4000, _drive.slideTop );
        sleep(1250);
        _drive.setGrip(true);
        sleep(500);
        //end of step two
        //start of step three
        _drive.setHeight(2500);
        sleep(1000);
        //Start of step four
        _drive.setExtension(960);
        // This function will start at the end of one cycle, turn around, grab a cone, and put it on the pole
    }

    private void scoreCone(SampleMecanumDrive _drive) {
        _drive.updatePoseEstimate();
        TrajectorySequence reposition = _drive.trajectorySequenceBuilder(stackPose)
                .turn(Math.toRadians(133), Math.toRadians(120), Math.toRadians(60))
                .build();


        _drive.setHeight(2500);
        _drive.setExtension(50);
        _drive.setSlideVelocity(4000, _drive.slideLeft, _drive.slideRight);

        _drive.followTrajectorySequence(reposition);

        sleep(250); // Wait for wiggles to stop just in case
        adjustAngle(_drive);

        // Increase the slide height to high junction height and increase its velocity //TODO:
        _drive.setSlideVelocity(4000, drive.slideLeft, drive.slideRight);
        _drive.setHeight(4200);
        // Wait until the slides are high enough that we won't hit the pole when extending
        sleep(500);
        // Extend the horizontal slide above the pole
        _drive.setSlideVelocity(4000, drive.slideTop);
        _drive.setExtension(850);

        // Wait for arm to be in position
        sleep(1500);

        // Open grip to drop cone
        _drive.setGrip(false);
        sleep(500);
    }

    private void parkBot(SampleMecanumDrive _drive, int _zone, Pose2d[] locations) {
        _drive.updatePoseEstimate();
        Trajectory moveToPark = _drive.trajectoryBuilder(_drive.getPoseEstimate())
                .lineToLinearHeading(locations[_zone])
                .build();

        _drive.setGrip(false);
        _drive.setExtension(50);
        _drive.setHeight(4400);
        _drive.setSlideVelocity(4000, _drive.slideLeft, _drive.slideRight, _drive.slideTop);

        _drive.followTrajectory(moveToPark);

        _drive.setHeight(100);
    }
}
*/