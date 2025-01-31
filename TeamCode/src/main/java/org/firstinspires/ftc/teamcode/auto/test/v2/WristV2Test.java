package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Wristv2;


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
                        wrist.wristFoldOutAction(),
                        new SleepAction(1), // sleep
                        wrist.wristFoldInAction(),
                        new SleepAction(1) // sleep
                ));

    } // runOpMode



}

