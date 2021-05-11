package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    //Trajectories
    Trajectory trajA1;
    Trajectory trajC1;
    Trajectory trajC2;
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

        //Case C:
        trajC1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(58,48))
                .build();

        trajC2 = drive.trajectoryBuilder(trajC1.end())
                .lineToConstantHeading(new Vector2d(-9, 48))
                .build();

        waitForStart();
        if (isStopRequested()) return;
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
        sleep(2000);
    }

    private void pathC() {
        drive.followTrajectory(trajC1);
        drive.wobbleDrop();
        sleep(2000);
        drive.followTrajectory(trajC2);
    }

}
