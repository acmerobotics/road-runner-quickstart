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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Disabled
@Autonomous(name = "Red Right Score", group = "competition")   //Telemtry allows us in the driver hub quickly chose which program to run
public class redRightScore extends LinearOpMode {
//extends linearOpMode allows us to call functions from other helper classes
    private final Pose2d startPose = new Pose2d(36, -64.25, Math.toRadians(90)); // our Starting pose allows us to know our postions of the robot and know what way it os looking at
    // later be called in our first trajectories

    //score pose is the x and y that our IMU tries to go too and the engocders goathers position data. The heading should looking at the nearest high junction
    private final Pose2d scorePose = new Pose2d(40, -11, Math.toRadians(141));

    // stack pose is what we point to when calling in our function so that we dont have to constantly put the same code in different trajectories. Stack pose
    // just slightly changes the postion of the robot while mainly just being a turn the change is y is for allowing the trjectory to build properly
    private final Pose2d stackPose = new Pose2d(40, -10, Math.toRadians(5));
// restrictions both in m/s
    private final double travelSpeed = 45.0, travelAccel = 30.0;
// the three different parking locations in poses
    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(64, -15, Math.toRadians(90))};
// camera images sizes 1280 pixles
    private final int width = 1280, height = 720;
// creates a dive object allows us to map funtions to our moters
    SampleMecanumDrive drive;

    // this is the microsoft life cam 3000
    OpenCvWebcam adjustCamera = null;

    // this is just our pipeline creating the filter of color on the signal sleeve
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone;

    @Override
    public void runOpMode() throws InterruptedException {   //when we start to run
        drive = new SampleMecanumDrive(hardwareMap);  // maps our moters to the robot

        // Initialize arm
        drive.initArm();

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts
// this is a trajectory we are telling the robot when goToStack is called to go from our stack pose to the score pose in a spline that looks like an s
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(scorePose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        // Set up the webcam
        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class, "adjustCamera");
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

        adjustCamera.stopStreaming();
        adjustCamera.closeCameraDevice();

        drive.setSlideVelocity(4000, drive.slideRight, drive.slideLeft, drive.slideTop);

        // Close the grip and move the slide up a small amount
        drive.setGrip(true);
        sleep(250);
        drive.setHeight(200);
        drive.setExtension(50);

        // The sleep is necessary to wait for certain arm actions to finish
        sleep(250);

        // Increase the height of the slide and increase its velocity
        drive.setHeight(4200);
        drive.setExtension(750);

        drive.followTrajectorySequence(goToStack);

        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location

        // Update roadrunner's idea of where the robot is after we ran the trajectory
        drive.updatePoseEstimate();
        // Wait for arm to be in position
        sleep(250);

        // Open grip to drop cone
        drive.setGrip(false);

        // Wait for grip to fully open and cone to drop
        sleep(500);
        // for liip to repeate 3 timss
        // calles totrack fuction that turns around grabs a cone and then stops
        // then calls score cone that goes from stack to target junction
        for (int i = 5; i > 2; i--) {
            toStack(drive, i);
            scoreCone(drive, i);
        }
// this is a qick if else statements that just calls the parkbot fucntion that just parks our bot using the zone from a list of the different parking postion
        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }
    }

    private void toStack(SampleMecanumDrive _drive, int stackHeight ) {
        // stackHeight is given as height of stack in cones
        //step one

        // we have to do _drive because if we just did drive we would run into a localization error.

        // this is after we drop we pull in the claw
        _drive.setExtension(500);

        // we wait for the claw to be pulled back becuase if we dont we would tunr and pull the junction this is also to reduce our radius which reduces
        // our roational intertia this is a principle tought in phyics classes higher the rotational inertia the harder it is to turn and to stop turning

        sleep(200);
// when hard coding we having problems with correct radians for the first turn to the stack we have to add degrees to compensate this problem
//we just have to call get pose esitment to allow our roboto to understnad where it is and then whrwr it wants to go.
        _drive.updatePoseEstimate();
        TrajectorySequence turnToStack = _drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                .addTemporalMarker(0.5, () -> {
                    _drive.setHeight(120 + (stackHeight * 145));
                })
                .addTemporalMarker(1, () -> {
                    _drive.setExtension(1850);
                })
                .turn(Math.toRadians(-154), Math.toRadians(120), Math.toRadians(90))
                .build();

        _drive.followTrajectorySequence(turnToStack);

        // claw moves out to grab a cone from the stack
        _drive.setExtension(2100);

        //we wait because if we dont then the claw closes before we can grip a cone
        sleep(750);
// cone is grab
        _drive.setGrip(true);
        sleep(450);
        //this is us now lifting the cone

        _drive.setHeight(4100);
        sleep(350);
        //pull back before we turn
        _drive.setExtension(800);
    }

    private void scoreCone(SampleMecanumDrive _drive, int stackHeight) {

        //trajectory to turn to target junction
        _drive.updatePoseEstimate();
        TrajectorySequence reposition = _drive.trajectorySequenceBuilder(stackPose)
                .turn(Math.toRadians(146), Math.toRadians(120), Math.toRadians(90))
                .build();
// just set the height of the claw
        _drive.setHeight(4100);
//we start to turn
        _drive.followTrajectorySequence(reposition);

        // we push out our arm 
        _drive.setExtension(800);

        // Wait for wiggles to stop just in case
        sleep(250);

        // Open grip to drop cone
        _drive.setGrip(false);
        sleep(250);
    }

    // This code parks our robot using a list of locations and zones. If you were to use encoders
    // it would work a bit differently, but i hope this helps
    private void parkBot(SampleMecanumDrive _drive, int _zone, Pose2d[] locations) {
        _drive.updatePoseEstimate();
        Trajectory moveToPark = _drive.trajectoryBuilder(_drive.getPoseEstimate())
                .lineToLinearHeading(locations[_zone])
                .build();

        _drive.setGrip(false);
        _drive.setExtension(50);
        _drive.setHeight(4400);

        _drive.followTrajectory(moveToPark);

        _drive.setHeight(100);
    }
}
*/