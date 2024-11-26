package org.firstinspires.ftc.teamcode.auto.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Arm;


@Config
@Autonomous(name = "ArmTest", group = "Test")
public class ArmTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Arm arm = new Arm(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.armScoreAction(),
                        new SleepAction(1), // sleep for 1 sec
                        arm.armfoldbackaction()));

    } // runOpMode



}

