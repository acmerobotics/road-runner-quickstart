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
    Trajectory wallOffset, trajPower1, trajPower2, trajPower3, trajShoot, trajParkA, trajParkB;
    //milliseconds of time to offset instructions
    // 1 second = 1000 milliseconds
    long waitOffset = 1000;
    //shooterVelocity
    int powerVel = (int)drive.powerVel;

    Vector2d wallOffPosition = new Vector2d(-58, 21);
    Vector2d power1Position = new Vector2d(-58, 29);
    Vector2d power2Position = new Vector2d(-58, 20);
    Vector2d power3Position = new Vector2d(-58, 17);
    Vector2d parkPosition = new Vector2d(12, 12);

    Pose2d power1Pose = new Pose2d(power1Position.getX(), power1Position.getY(), Math.toRadians(5));
    Pose2d parkPose = new Pose2d(parkPosition.getX(), parkPosition.getY(), Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        //Start Position
        Pose2d startPose = new Pose2d(-63, 24, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Trajectories

        wallOffset = drive.trajectoryBuilder(startPose)
                .lineTo(wallOffPosition)
                .build();

        trajPower1 = drive.trajectoryBuilder(wallOffset.end())
                .lineToSplineHeading(power1Pose)
                .build();

        trajPower2 = drive.trajectoryBuilder(trajPower1.end())
                .strafeTo(power2Position)
                .build();

        trajPower3 = drive.trajectoryBuilder((trajPower2.end()))
                .strafeTo(power3Position)
                .build();

        trajParkA = drive.trajectoryBuilder(trajPower3.end())
                .lineToSplineHeading(new Pose2d(trajPower3.end().getX(), parkPose.getY(), parkPose.getHeading()))
                .build();

        trajParkB = drive.trajectoryBuilder(trajParkA.end())
                .strafeTo(new Vector2d(parkPosition.getX(), parkPosition.getY()))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //Actual Movement
        drive.moveTo("Away");
        sleep(waitOffset);
        drive.prepShooter(powerVel);
        drive.spinIntake();
        //Move off the wall
        drive.followTrajectory(wallOffset);
        //Line up for first PowerShot
        drive.followTrajectory(trajPower1);
        drive.shootRings(1, powerVel, true);
        sleep(100);
        //Line up for second PowerShot
        drive.followTrajectory(trajPower2);
        sleep(100);
        drive.shootRings(1, powerVel, true);
        //Line up for third PowerShot
        drive.followTrajectory(trajPower3);
        sleep(100);
        drive.shootRings(1, powerVel, false);
        //Park
        drive.followTrajectory(trajParkA);
        drive.followTrajectory(trajParkB);
    }
}