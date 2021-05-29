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
public class Auton_RedField_ShootPark extends LinearOpMode {
    //We have an issue with using the same auton for both sides. The start positions are different, and that could lead to potential issues.
    private Servo wobbleDropper;
    SampleMecanumDrive drive;
    Trajectory trajShoot, trajParkA, trajParkB;
    //milliseconds of time to offset instructions
    // 1 second = 1000 milliseconds
    long waitOffset = 1000;

    Vector2d shootPosition = new Vector2d(-10, -26);
    Vector2d parkPosition = new Vector2d(12, 12);
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //Start Position
        Pose2d startPose = new Pose2d(-63, -24, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Trajectories
        trajShoot = drive.trajectoryBuilder(startPose)
                .strafeTo(shootPosition)
                .build();

        trajParkA = drive.trajectoryBuilder(trajShoot.end())
                .strafeTo(new Vector2d(shootPosition.getX(), parkPosition.getY()))
                .build();

        trajParkB = drive.trajectoryBuilder(trajParkA.end())
                .strafeTo(new Vector2d(parkPosition.getX(), parkPosition.getY()))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //Actual Movement
        drive.moveTo("Away");
        sleep(waitOffset);
        drive.prepShooter();
        drive.followTrajectory(trajShoot);
        drive.shootRings(3);
        drive.followTrajectory(trajParkA);
        drive.followTrajectory(trajParkB);
    }
}