package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Liftv2;

@Config
@Autonomous(name = "LiftV2 Test", group = "Test")
public class LiftV2Test extends LinearOpMode {



    @Override
    public void runOpMode() {

        Liftv2 lift = new Liftv2(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        lift.liftUpAction(),
                        new SleepAction(1),
                        lift.liftDownAction()));
    }



}
