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
public class Auton_BlueField_PowerPark extends LinearOpMode {
    //We have an issue with using the same auton for both sides. The start positions are different, and that could lead to potential issues.
    private Servo wobbleDropper;
    SampleMecanumDrive drive;
    Trajectory trajPower1, trajPower2, trajPower3, trajShoot, trajParkA, trajParkB;
    //milliseconds of time to offset instructions
    // 1 second = 1000 milliseconds
    long waitOffset = 1000;
    int targetVel = 2300;

    Vector2d power1Position = new Vector2d(-63, 29);
    Vector2d power2Position = new Vector2d(-63, 23);
    Vector2d power3Position = new Vector2d(-63, 17);
    Vector2d parkPosition = new Vector2d(12, 12);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //Start Position
        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Trajectories

        trajPower1 = drive.trajectoryBuilder(startPose)
                .strafeTo(power1Position)
                .build();

        trajPower2 = drive.trajectoryBuilder(trajPower1.end())
                .strafeTo(power2Position)
                .build();

        trajPower3 = drive.trajectoryBuilder((trajPower2.end()))
                .strafeTo(power3Position)
                .build();

        trajParkA = drive.trajectoryBuilder(trajShoot.end())
                .strafeTo(new Vector2d(trajPower3.end().getX(), parkPosition.getY()))
                .build();

        trajParkB = drive.trajectoryBuilder(trajParkA.end())
                .strafeTo(new Vector2d(parkPosition.getX(), parkPosition.getY()))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //Actual Movement
        drive.moveTo("Away");
        sleep(waitOffset);
        drive.prepShooter(targetVel);
        //Line up for first PowerShot
        drive.followTrajectory(trajPower1);
        drive.spinIntake();
        drive.shootRings(1, targetVel, true);
        //Line up for second PowerShot
        drive.followTrajectory(trajPower2);
        drive.shootRings(1, targetVel, true);
        //Line up for third PowerShot
        drive.followTrajectory(trajPower3);
        drive.shootRings(1, targetVel, false);
        //Park
        drive.followTrajectory(trajParkA);
        drive.followTrajectory(trajParkB);
    }
}