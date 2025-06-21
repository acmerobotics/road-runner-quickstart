package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;/*package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay.poleFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Config
@Autonomous(name = "RedEXTENDED w/ Camera")
public class redRightScoreCamEXTENDED extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(39, -63, Math.toRadians(90));
    private final Pose2d stackPose = new Pose2d(36, -12, Math.toRadians(90));

    private final double travelSpeed = 45.0, travelAccel = 20.0;
    private final double adjustmentSpeed = 1.5, adjustmentAccel = 1.5;
    private final double angVel = Math.toRadians(180), adjustAngVel = Math.toRadians(20);

    private final int width = 1280, height = 720, slices = 64;

    SampleMecanumDrive drive;
    OpenCvWebcam camera = null;
    // poleFinder poleFinderPipeline = new poleFinder(width, height, slices);
    poleFinder poleFinderPipeline = new poleFinder();
    poleFinder.poleLocation moveDir;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize arm
        drive.initArm();

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(3)
                .lineToLinearHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                //.turn(-Math.atan2(drive.getPoseEstimate().getY()-24, drive.getPoseEstimate().getX()-0) - Math.toRadians(drive.getPoseEstimate().getHeading()))
                .turn(Math.toRadians(45))
                .build();

        // Set up the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Set the camera's pipeline
        camera.setPipeline(poleFinderPipeline);

        // Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        // Close the grip and move the slide up a small amount
        drive.setGrip(true);
        drive.setHeight(200);
        drive.setExtension(0);
        drive.setSlideVelocity(400, drive.slideLeft, drive.slideRight);

        // The sleep is necessary to wait for certain arm actions to finish
        sleep(250);

        // Increase the height of the slide and increase its velocity
        drive.setHeight(2500);
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight);

        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location
        drive.followTrajectorySequence(goToStack);
        // Update roadrunner's idea of where the robot is after we ran the trajectory
        drive.updatePoseEstimate();
        // Adjust our angle so that we are lined up with the pole
        adjustAngle(drive);

        // Increase the slide height to high junction height and increase its velocity
        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        drive.setHeight(4200);
        // Wait until the slides are high enough that we won't hit the pole when extending
        sleep(500);
        // Extend the horizontal slide above the pole
        drive.setSlideVelocity(1000, drive.slideTop);
        drive.setExtension(700);

        // Wait for arm to be in position
        sleep(1500);

        // Open grip to drop cone
        drive.setGrip(false);

        // Wait for grip to fully open and cone to drop
        sleep(500);

        // Drop and retract slides to cycle the next cone
        drive.setExtension(100);
        drive.setHeight(100);
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight, drive.slideTop);

        // Update roadrunner's idea of our location after we adjusted it
        drive.updatePoseEstimate();
        // Turn around to pick up the next cone
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(40, -10, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .build()
        );
        // Update our pose again
        drive.updatePoseEstimate();

        // Reach out for the cone from the stack
        drive.setHeight(750);
        drive.setExtension(1950);
        // Wait until grip is in position
        sleep(1500);
        // Close grip
        drive.setGrip(true);
        // Wait for grip to be fully closed
        sleep(500);

        // Raise vertical slides to the correct height
        drive.setHeight(2500);
        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        // Wait until cone is completely off the stack before pulling the horizonal slide in
        sleep(500);
        drive.setExtension(0);

        // Turn back around to face the pole
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(135)),
                        SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .build()
        );

        // Update our position
        drive.updatePoseEstimate();
        // Adjust our angle
        adjustAngle(drive);

        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        drive.setHeight(4200);
        sleep(500);
        drive.setSlideVelocity(1000, drive.slideTop);
        drive.setExtension(700);
//where it lets go
        sleep(1000);
        drive.setGrip(false);
        sleep(500);
        drive.setExtension(100);
        drive.setHeight(100);
        drive.setSlideVelocity(1000, drive.slideLeft, drive.slideRight, drive.slideTop);

        // Update roadrunner's idea of our location after we adjusted it
        drive.updatePoseEstimate();
        // Turn around to pick up the next cone
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(40, -10, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .build()
        );
        // Update our pose again
        drive.updatePoseEstimate();

        // Reach out for the cone from the stack
        drive.setHeight(750);
        drive.setExtension(1950);
        // Wait until grip is in position
        sleep(1500);
        // Close grip
        drive.setGrip(true);
        // Wait for grip to be fully closed
        sleep(500);

        // Raise vertical slides to the correct height
        drive.setHeight(2500);
        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        // Wait until cone is completely off the stack before pulling the horizonal slide in
        sleep(500);
        drive.setExtension(0);

        // Turn back around to face the pole
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(135)),
                        SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(40), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20)
                )
                .build()
        );

        // Update our position
        drive.updatePoseEstimate();
        // Adjust our angle
        adjustAngle(drive);

        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        drive.setHeight(4200);
        sleep(500);
        drive.setSlideVelocity(1000, drive.slideTop);
        drive.setExtension(700);
//where it lets go
        sleep(1000);
        drive.setGrip(false);
        sleep(500);
        drive.setSlideVelocity(1000, drive.slideTop);
        drive.setExtension(0);
        drive.setSlideVelocity(2000, drive.slideLeft, drive.slideRight);
        drive.setHeight(0);
        sleep(500);
//where me and eli need to start
        while (true) {
            if (isStopRequested()) {
                break;
            }
        }
    }

    private void adjustAngle(SampleMecanumDrive _drive) {
        double degrees = 2.0;
        int tries = 1;
        poleFinder.poleLocation prev = poleFinder.poleLocation.ALIGNED;

        while (moveDir != poleFinder.poleLocation.ALIGNED && tries > 0) {
            moveDir = poleFinderPipeline.getLocation();
            if (moveDir == poleFinder.poleLocation.LEFT) {
                if (prev == moveDir) {
                    tries -= 1;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(degrees))
                        .build());
                prev = poleFinder.poleLocation.LEFT;
            } else if (moveDir == poleFinder.poleLocation.RIGHT) {
                if (prev == moveDir) {
                    tries -= 1;
                    degrees -= 1.0;
                }
                _drive.followTrajectorySequence(_drive.trajectorySequenceBuilder(_drive.getPoseEstimate())
                        .turn(Math.toRadians(-degrees))
                        .build());
                prev = poleFinder.poleLocation.RIGHT;
            }
            if (isStopRequested()) {
                break;
            }
        }
    }

    private void cycleCone(SampleMecanumDrive _drive) {
        // This function will start at the end of one cycle, turn around, grab a cone, and put it on the pole
    }
}
*/