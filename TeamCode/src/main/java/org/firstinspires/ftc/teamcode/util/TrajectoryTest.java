package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory goForward = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(20) //.forward(x) go forward x inches; .back(x) go back x inches
                .build();
        drivetrain.followTrajectory(goForward);



        Trajectory lineToPosition = drivetrain.trajectoryBuilder(new Pose2d(10,0,0))
                .lineTo(new Vector2d(0,0))  //vector does not care about heading, holds current heading 0
                .build();
        drivetrain.followTrajectory(lineToPosition);


        Trajectory strafeLeft = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeLeft(10)
                .build();
        drivetrain.followTrajectory((strafeLeft));

        Trajectory strafeToPosition = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(0,10))
                .build();
        drivetrain.followTrajectory(strafeToPosition);

/**
        Trajectory splineToPosition = drivetrain.trajectoryBuilder((new Pose2d(0,0, Math.toRadians(90))))
                .splineTo(new Pose2d(50,50,Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(splineToPosition);
/**
        while opModeIsActive(){

        }
 */
    }


}
