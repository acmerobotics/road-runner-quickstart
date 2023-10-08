package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous (name="Robot: Auto Test--BE CAREFUL!!", group="Robot")
public class AutoTest extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    public Action trajectory =
            drive.actionBuilder(drive.pose)
                    .lineToYConstantHeading(20)   //TODO: lineTo vs splineTo??
                    .turn(Math.PI / 2)
                    .lineToX(10)

                    .build();
    //TODO: lineToConstantHeading vs lineTo?

    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            runBlocking(trajectory); //TODO: make brainSTEM robot class
        }
    }
}