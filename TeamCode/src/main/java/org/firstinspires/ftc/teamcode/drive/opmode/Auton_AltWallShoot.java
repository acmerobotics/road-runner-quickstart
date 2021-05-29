package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 * This is a straight line autonomous that will drop 1 wobble goal at square A on both sides of the field
 */

@Autonomous(group = "drive")
public class Auton_AltWallShoot extends LinearOpMode {
    //We have an issue with using the same auton for both sides. The start positions are different, and that could lead to potential issues.
    private Servo wobbleDropper;
    SampleMecanumDrive drive;
    Trajectory traj0, traj1, traj2, traj3;
    //milliseconds of time to offset instructions
    // 1 second = 1000 milliseconds
    long waitOffset = 10000;
    int targetVel = 2350;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //Start Position
        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Trajectories
        traj0 = drive.trajectoryBuilder(startPose)
                //.strafeTo(new Vector2d(, ));
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //Actual Movement
        drive.moveTo("Away");
        sleep(waitOffset);
        drive.followTrajectory(traj0);
        drive.shootRings(3, targetVel);
    }
}