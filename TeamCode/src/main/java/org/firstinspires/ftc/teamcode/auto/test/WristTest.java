package org.firstinspires.ftc.teamcode.auto.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Wrist;


@Config
@Autonomous(name = "IntakeTest", group = "Test")
public class WristTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Wrist wrist = new Wrist(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

//        Actions.runBlocking(
//                new SequentialAction(
//                        wrist.foldIn(),
//                        new SleepAction(1)) // sleep for 1 sec
//                        wrist.foldOut());

    } // runOpMode



}

