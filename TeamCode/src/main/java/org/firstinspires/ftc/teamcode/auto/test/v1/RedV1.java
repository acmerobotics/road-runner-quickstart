package org.firstinspires.ftc.teamcode.auto.test.v1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "\uD83D\uDD34 - Red V1", group = "RoadRunner 1.0")
@Disabled
public class RedV1 extends LinearOpMode {


    // Start position red near
    Pose2d RED_NEAR_START_POSE = new Pose2d(12, -60, Math.PI/2.0);


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_NEAR_START_POSE);

        // Set to true when an AprilTag target is detected
        boolean targetFound = false;

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


    } // runOpMode



}