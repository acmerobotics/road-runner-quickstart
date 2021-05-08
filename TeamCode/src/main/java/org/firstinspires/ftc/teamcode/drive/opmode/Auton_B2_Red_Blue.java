package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/*
 * This is a complex autonomous that will score 2 wobble goals in a complex path on square B
 */
@Autonomous(group = "drive")
public class Auton_B2_Red_Blue extends LinearOpMode {
    //We have an issue with using the same auton for both sides. The start positions are different, and that could lead to potential issues.
    private Servo wobbleDropper;
    private double slowerVelocity = 5;
    private DcMotor wobbleArm;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        wobbleDropper = hardwareMap.get(Servo.class, "wobbleDropper");
        TFObjectDetector tfod = drive.getTfod();
        wobbleArm = drive.getWobbleArm();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        waitForStart();

        if (isStopRequested()) return;
//        drive.moveTo("Away");
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-60, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-24, 48))
                .splineTo(new Vector2d(36, 19), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-18, 19))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(
                new Vector2d(-27, 19),
                SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(20, 24))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(-135))), false)
                .forward(15)
                .build();
//      Move to block b
        drive.followTrajectory(traj1);
        drive.moveTo("Down");
        drive.wobbleDrop();
//        drive.moveTo("Away");
        sleep(1500);
        //Go get second wobble
        drive.followTrajectory(traj2);
        //Move to second wobble
        drive.followTrajectory(traj3);
        // Grab
        drive.wobbleGrab();
        sleep(2000);
        drive.moveTo("Carry");
        drive.followTrajectory(traj4);
        drive.turn(Math.toRadians(-135));
        drive.moveTo("Down");
        // Drop Wobble from arm
        drive.wobbleRelease();
        sleep(1000);
        // Go Park
        drive.followTrajectory(traj5);
//        drive.moveTo("Away");
//        sleep(2000);

//        wobbleDropper.setPosition(1);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}
