package org.firstinspires.ftc.teamcode.auto.test.v1;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.v1.Wrist;


@Config
@Autonomous(name = "WristTest", group = "Test")
public class WristTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Wrist wrist = new Wrist(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        wrist.wristFoldOutAction(),
                        new SleepAction(1), // sleep for 1 sec
                        wrist.wristFoldInAction(),
                        new SleepAction(1),// sleep for 1 sec
                        wrist.wristFoldOutAction(),
                        new SleepAction(1), // sleep for 1 sec
                        wrist.wristFoldInAction(),
                        new SleepAction(1),// sleep for 1 sec
                        wrist.wristFoldOutAction(),
                        new SleepAction(1), // sleep for 1 sec
                        wrist.wristFoldInAction(),
                        new SleepAction(1)// sleep for 1 sec
                ));

    } // runOpMode



}

