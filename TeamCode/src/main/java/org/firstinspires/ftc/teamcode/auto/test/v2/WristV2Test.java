package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Wristv2;


@Config
@Autonomous(name = "WristV2 Test", group = "Test")
public class WristV2Test extends LinearOpMode {



    @Override
    public void runOpMode() {

        Wristv2 wrist = new Wristv2(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        wrist.wristPickUpGroundSampleAction(),
                        new SleepAction(4), // sleep for 1 sec
                        wrist.wristFoldOutAction(),
                        new SleepAction(4) // sleep for 1 sec
                ));

    } // runOpMode



}

