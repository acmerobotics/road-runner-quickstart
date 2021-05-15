package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

/*
 * This is a straight line autonomous that will drop 1 wobble goal at square A on both sides of the field
 */

@Autonomous(group = "drive")
public class Auton_Blue_Wall extends LinearOpMode {
    //We have an issue with using the same auton for both sides. The start positions are different, and that could lead to potential issues.
    private Servo wobbleDropper;
    private double slowerVelocity = 3;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //TODO: Make an Interface to make these repeated portions of code inheritable
    //Initializing Trajectories
    Trajectory trajA1;
    Trajectory trajA2;
    Trajectory trajA3;
    Trajectory trajA4;

    Trajectory trajB1;
    Trajectory trajB2;
    Trajectory trajB3;
    Trajectory trajB4;
    Trajectory trajB5;

    Trajectory trajC1;
    Trajectory trajC2;
    Trajectory trajC3;
    Trajectory trajC4;
    @Override
    public void runOpMode() throws InterruptedException {

        // Starting Position
        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        //Init trajectories

        //Case A:
        trajA1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-9,48))
                .build();

        trajA2 = drive.trajectoryBuilder(trajA1.end())
                .lineToConstantHeading(new Vector2d(-9, 19))
                .lineToConstantHeading(new Vector2d(-18,19))
                .build();

        trajA3 = drive.trajectoryBuilder(trajA2.end())
                .lineTo(
                        new Vector2d(-27, 19),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        trajA4 = drive.trajectoryBuilder(trajA3.end())
                .lineToConstantHeading(new Vector2d(-9, 19))
                .build();

        //Case B:
        trajB1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-24, 48))
                .splineTo(new Vector2d(36, 19), Math.toRadians(0))
                .build();

        trajB2 = drive.trajectoryBuilder(trajB1.end())
                .lineToConstantHeading(new Vector2d(-18, 19))
                .build();

        trajB3 = drive.trajectoryBuilder(trajB2.end())
                .lineTo(
                        new Vector2d(-27, 19),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        trajB4 = drive.trajectoryBuilder(trajB3.end())
                .lineToSplineHeading(new Pose2d(20, 19, Math.toRadians(-135)))
                .build();

        trajB5 = drive.trajectoryBuilder(trajB4.end(), false)
                .forward(6)
                .build();

        //Case C:
        trajC1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(58,48))
                .build();

        trajC2 = drive.trajectoryBuilder(trajC1.end())
                .lineToConstantHeading(new Vector2d(58, 19))
                .lineToConstantHeading(new Vector2d(-18,19))
                .build();

        trajC3 = drive.trajectoryBuilder(trajC2.end())
                .lineTo(new Vector2d(-27, 19),
                SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
        )
                .build();

        trajC4 = drive.trajectoryBuilder(trajC3.end())
                .lineToConstantHeading(new Vector2d(-9, 19))
                .build();


        waitForStart();
        if (isStopRequested()) return;
        //Use Tensorflo to figure out which path to use
        //Test all the paths.

        pathA();
//        pathB();
//        pathC();

//        wobbleDropper.setPosition(1);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );


    }

    private void pathA() {
        drive.followTrajectory(trajA1);
        drive.wobbleDrop();
        sleep(2000);
        drive.followTrajectory(trajA2);
        drive.followTrajectory(trajA3);
        //grab wobbleGoal
        drive.followTrajectory(trajA4);
    }

    private void pathB() {
        drive.followTrajectory(trajB1);
        drive.wobbleDrop();
        sleep(2000);
        drive.followTrajectory(trajB2);
        drive.followTrajectory(trajB3);
        //grab wobbleGoal
        sleep(2000);
        drive.followTrajectory(trajB4);
        drive.followTrajectory(trajB5);
    }

    private void pathC() {
        drive.followTrajectory(trajC1);
        drive.wobbleDrop();
        sleep(2000);
        drive.followTrajectory(trajC2);
        drive.followTrajectory(trajC3);
        //grab wobbleGoal
        drive.followTrajectory(trajC4);
    }

}
