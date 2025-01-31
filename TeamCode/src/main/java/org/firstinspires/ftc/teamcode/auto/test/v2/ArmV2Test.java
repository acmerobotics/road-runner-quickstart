package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Armv2;


@Autonomous(name = "ArmV2 Test", group = "Test")
public class ArmV2Test extends LinearOpMode {



    @Override
    public void runOpMode() {

        Armv2 arm = new Armv2(hardwareMap);
        arm.reset();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        arm.armPickupGroundSampleAction(),
                        new SleepAction(4), // sleep
                        arm.armVerticalAction(),
                        new SleepAction(4),
                        arm.armResetAction()
                        ));

    } // runOpMode



}

