package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Armv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Wristv2;


@Config
@Autonomous(name = "Sample Pick Test", group = "Test")
public class SamplePickTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Wristv2 wrist = new Wristv2(hardwareMap);
        Armv2 arm = new Armv2(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        arm.reset();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        claw.clawOpenAction(),
                        wrist.wristPickUpGroundSampleAction(),
                        arm.armPickupGroundSampleAction(),
                        new SleepAction(4), // sleep
                        claw.clawCloseAction(),
                        new SleepAction(1),
                        wrist.wristFoldOutAction(),
                        arm.armResetAction(),
                        new SleepAction(4)
                ));

    } // runOpMode



}

