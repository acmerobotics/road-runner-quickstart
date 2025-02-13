package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.ClawRotator;


@Autonomous(name = "ClawRotatorTest", group = "Test")
public class ClawRotatorTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        ClawRotator clawRotator = new ClawRotator(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        clawRotator.clawRotate45LeftAction(),
                        new SleepAction(2),
                        clawRotator.clawRotateResetAction(),
                        new SleepAction(2),
                        clawRotator.clawRotate45RightAction(),
                        new SleepAction(2),
                        clawRotator.clawRotateResetAction(),
                        new SleepAction(2) 
                ));

    } // runOpMode
}

