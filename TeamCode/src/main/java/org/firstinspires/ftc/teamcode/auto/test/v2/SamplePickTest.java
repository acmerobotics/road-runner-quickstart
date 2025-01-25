package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Robotv2;


@Config
@Autonomous(name = "Sample Pick Test", group = "Test")
public class SamplePickTest extends LinearOpMode {



    @Override
    public void runOpMode() {

        Robotv2 robot = new Robotv2(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));

        robot.arm.reset();
        robot.lift.reset();


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                robot.claw.clawOpenAction()
                                , robot.lift.liftUpAction()
                                , robot.wrist.wristVerticalAction()
                        )
                        , new SleepAction(1)
                        , robot.comeDownAndPickUpActionAuto()
                        , new SleepAction(1)
                        , robot.scoreSampleActionAuto()
                        , new SleepAction(1)
                        , robot.comeDownAndPickUpActionAuto()
                        , new SleepAction(1)
                        , robot.scoreSampleActionAuto()
                        , new SleepAction(1)
                        , robot.resetAction()
                        , new SleepAction(1)
                )
        );


    } // runOpMode



}

