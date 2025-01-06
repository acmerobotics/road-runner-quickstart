package org.firstinspires.ftc.teamcode.auto.test.v1;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.v1.Intake;


@Config
@Autonomous(name = "IntakeTest", group = "Test")
@Disabled
public class IntakeTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Intake intake = new Intake(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        intake.depositAction(),
                        new SleepAction(1), // sleep for 1 sec
                        intake.intakeAction()));

    } // runOpMode



}

