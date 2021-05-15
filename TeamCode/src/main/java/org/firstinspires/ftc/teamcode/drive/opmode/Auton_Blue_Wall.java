package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor wobbleArm;
    SampleMecanumDrive drive;

    //TODO: Make an Interface to make these repeated portions of code inheritable
    //Initializing Trajectories
    Trajectory trajA1;
    Trajectory trajA2;
    Trajectory trajA3;
    Trajectory trajA4;
    Trajectory trajA5;
    Trajectory trajA6;
    Trajectory trajA7;

    Trajectory trajB1;
    Trajectory trajB2;
    Trajectory trajB3;
    Trajectory trajB4;
    Trajectory trajB5;

    Trajectory trajC1;
    Trajectory trajC2;
    Trajectory trajC3;
    Trajectory trajC4;
    Trajectory trajC5;
    Trajectory trajC6;
    Trajectory trajC7;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // Starting Position
        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        //Init trajectories

        //Case A:
        trajA1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(12, 48))
                .build();

        trajA2 = drive.trajectoryBuilder(trajA1.end())
                .lineToConstantHeading(new Vector2d(12, 19))
                .build();

        trajA3 = drive.trajectoryBuilder(trajA2.end())
                .lineToConstantHeading(new Vector2d(-18, 19))
                .build();

        trajA4 = drive.trajectoryBuilder(trajA3.end())
                .lineTo(new Vector2d(-27, 19),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        trajA5 = drive.trajectoryBuilder(trajA4.end())
                .lineToConstantHeading(new Vector2d(12, 19))
                .build();

        trajA6 = drive.trajectoryBuilder(trajA5.end())
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(-90)))
                .build();

        trajA7 = drive.trajectoryBuilder(trajA6.end(), false)
                .forward(6)
                .build();

        //Case B:
        trajB1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-24, 48))
                .splineTo(new Vector2d(36, 19), Math.toRadians(0))
                .build();

        trajB2 = drive.trajectoryBuilder(trajB1.end())
                .strafeTo(new Vector2d(-18, 19))
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
                .lineTo(new Vector2d(56, 48))
                .build();

        trajC2 = drive.trajectoryBuilder(trajC1.end(), true)
                .splineTo(new Vector2d(-18, 19), Math.toRadians(180))
                .build();

        trajC3 = drive.trajectoryBuilder(trajC2.end())
                .lineTo(new Vector2d(-27, 19),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        trajC4 = drive.trajectoryBuilder(trajC3.end())
                .strafeTo(new Vector2d(56, 19))
                .build();

        trajC5 = drive.trajectoryBuilder(trajC4.end())
                .lineToLinearHeading(new Pose2d(56, 36, Math.toRadians(-90)))
                .build();

        trajC6 = drive.trajectoryBuilder(trajC5.end(), false)
                .strafeTo(new Vector2d(56, 30))
                .build();

        trajC7 = drive.trajectoryBuilder(trajC6.end(), false)
                .strafeTo(new Vector2d(12, 30))
                .build();



        waitForStart();
        if (isStopRequested()) return;
        //Use Tensorflo to figure out which path to use
        //Test all the paths.

//        pathA();
//        pathB();
        pathC();

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
//        drive.moveTo("Down");
        sleep(2000);
        drive.followTrajectory(trajA2);
        drive.followTrajectory(trajA3);
        drive.followTrajectory(trajA4);
//        drive.wobbleGrab();
        sleep(2000);
//        drive.moveTo("Carry");
        drive.followTrajectory(trajA5);
        drive.followTrajectory(trajA6);
//        drive.moveTo("Down");
//        drive.wobbleRelease();
        sleep(2000);
        drive.followTrajectory(trajA7);
        sleep(2000);
//        drive.moveTo("Away");
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
        drive.followTrajectory(trajC2);
        drive.followTrajectory(trajC3);
        drive.followTrajectory(trajC4);
        drive.followTrajectory(trajC5);
        drive.followTrajectory(trajC6);
        drive.followTrajectory(trajC7);
    }

}
