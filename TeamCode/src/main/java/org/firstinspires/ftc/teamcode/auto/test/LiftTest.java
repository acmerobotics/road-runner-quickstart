package org.firstinspires.ftc.teamcode.auto.test;


import static org.firstinspires.ftc.teamcode.mechanisms.Lift.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;


@Config
@Autonomous(name = "LiftTest", group = "Test")
public class LiftTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Lift lift = new Lift(hardwareMap);

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1), // sleep for 1 sec
                        lift.LiftScoreAction()));
    }



}
