package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay; /**package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.parkingZoneFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous (name= "RightMEDStack", group = "comepetition")
public class Revamp extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(35, -64.25, Math.toRadians(90)); // our Starting pose allows us to know our postions of the robot and know what way it os looking at
    // later be called in our first trajectories

    //score pose is the x and y that our IMU tries to go too and the engocders goathers position data. The heading should looking at the nearest high junction
    private final Pose2d highJun = new Pose2d(37, -11, Math.toRadians(145));

    // stack pose is what we point to when calling in our function so that we dont have to constantly put the same code in different trajectories. Stack pose
    // just slightly changes the postion of the robot while mainly just being a turn the change is y is for allowing the trjectory to build properly
    private final Pose2d stackPose = new Pose2d(48, -13, Math.toRadians(-5));
    // restrictions both in m/s

    private final Pose2d smalljun = new Pose2d(37, -13, Math.toRadians(290));

   // private final Pose2d medjun1=new Pose2d(1,-11, Math.toRadians(290));

    private final Pose2d medjun2= new Pose2d(38,-13, Math.toRadians(220));

    //orgianl 45 , 30
    private final double travelSpeed = 45.0, travelAccel = 30.0;
    // the three different parking locations in poses
    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(64, -15, Math.toRadians(90))};
    // camera images sizes 1280 pixles


    SampleMecanumDrive drive;

    private final int width = 1280, height = 720;

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
                .lineToSplineHeading(highJun,
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

        TrajectorySequence tostack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();


        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location

        // Update roadrunner's idea of where the robot is after we ran the trajectory

        // Wait for arm to be in position
        sleep(250);

        // Open grip to drop cone
        drive.setGrip(false);

        // Wait for grip to fully open and cone to drop
        sleep(500);



        //to stack is right
        drive.followTrajectorySequence(tostack);
        drive.setHeight(845);
        sleep(2000);
        drive.setGrip(true);

        drive.updatePoseEstimate();
                TrajectorySequence toSMALL = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(smalljun,
                                SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                        )
                        .build();

         drive.updatePoseEstimate();
                TrajectorySequence toMED2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(medjun2,
                                SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                        )
                        .build();


        //TODO THIS IS THE END OF THE FIRST CYCLE

        for(int i = 5; i>1;i--){
            if (i>3)
            {
                scoresmall(drive,i, toSMALL, tostack);
            }
            else
            {
                scoremed (drive,i,toMED2, tostack);
            }

        }
        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }




        //grab the 5th cone of the stack

        /*
        drive.updatePoseEstimate();

        for(int i=0;i>2;i++) {
            TrajectorySequence toSMALL = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(smalljun,
                            SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                    )
                    .build();


            drive.updatePoseEstimate();
            drive.followTrajectorySequence(toSMALL);
            drive.updatePoseEstimate();
            TrajectorySequence tostack2 = drive.trajectorySequenceBuilder(smalljun)
                    .lineToLinearHeading(stackPose,
                            SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                    )
                    .build();

            drive.setGrip(false);
            drive.setHeight(1500);
            sleep(2000);

            drive.updatePoseEstimate();
           drive.followTrajectorySequence(tostack2);

            drive.updatePoseEstimate();
        }


        TrajectorySequence toMED1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(medjun1,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        sleep(2000);
        drive.updatePoseEstimate();
        drive.followTrajectorySequence(toMED1);
        sleep(2000);
        drive.updatePoseEstimate();



        TrajectorySequence tostack3 = drive.trajectorySequenceBuilder(medjun1)
                .lineToLinearHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        drive.updatePoseEstimate();
        drive.followTrajectorySequence(tostack3);


        for(int i = 0 ; i<2 ; i++){
            sleep(2000);
            drive.updatePoseEstimate();
            TrajectorySequence toMED2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(medjun2,
                            SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                    )
                    .build();
            drive.followTrajectorySequence(toMED2);
            TrajectorySequence tostack4 = drive.trajectorySequenceBuilder(medjun2)
                    .lineToLinearHeading(stackPose,
                            SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                    )
                    .build();
            drive.updatePoseEstimate();
            drive.followTrajectorySequence(tostack4);
        }





        // for liip to repeate 3 timss
        // calles totrack fuction that turns around grabs a cone and then stops
        // then calls score cone that goes from stack to target junction
        // commented out

        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }



//  }

//  public void scoresmall(SampleMecanumDrive _drive, int h, TrajectorySequence small, TrajectorySequence stack){

//    _drive.setHeight(1500);

//    _drive.followTrajectorySequence(small);


//      _drive.followTrajectorySequence(stack);
//     _drive.setHeight(1350-(h*150));
//     sleep(500);
//      _drive.setGrip(true);
//      sleep(250);

//  }

// mj - middle junction
//  public void scoremed(SampleMecanumDrive _drive,int h,TrajectorySequence med,TrajectorySequence stack){
//      _drive.setHeight(2000);
//      _drive.updatePoseEstimate();

//     _drive.followTrajectorySequence(toMED2);
//     _drive.setGrip(false);
//            _drive.updatePoseEstimate();
       /* TrajectorySequence tostack4 = _drive.trajectorySequenceBuilder(medjun2)
                .lineToLinearHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
                       _drive.updatePoseEstimate();

        _drive.followTrajectorySequence(tostack4);
        _drive.setHeight(1050-(h*150));
        _drive.setGrip(false);
        sleep(250);

    }

    //high junction
    public void hj (){

    }

    // low junction
    public void lj() {

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