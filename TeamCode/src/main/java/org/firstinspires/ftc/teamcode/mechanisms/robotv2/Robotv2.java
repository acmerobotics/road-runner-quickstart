package org.firstinspires.ftc.teamcode.mechanisms.robotv2;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
public class Robotv2 {
    public MecanumDrive drive;
    public Armv2 arm;
    public Wristv2 wrist;
    public Liftv2 lift;
    public Claw claw;
    public ClawRotator clawRotator;
    public RightActuatorServo rightArmServo;
    public LeftActuatorServo leftArmServo;

    public LeftActuator leftActuator;
    public RightActuator rightActuator;

    public static double PRE_PICKUP_SLEEP = 1;
    public static double POST_PICKUP_SLEEP = 1;

    public static double PRE_DROP_SLEEP = 1;
    public static double POST_DROP_SLEEP = 1;
    public static double POST_DROP_SLEEP2 = 1;

    public Robotv2(HardwareMap hardwareMap, Pose2d startPose){
        drive = new MecanumDrive(hardwareMap, startPose);
        arm = new Armv2(hardwareMap);
        wrist = new Wristv2(hardwareMap);
        lift = new Liftv2(hardwareMap);
        claw = new Claw(hardwareMap);
        clawRotator = new ClawRotator(hardwareMap);

        // hang mechanism
        rightArmServo = new RightActuatorServo(hardwareMap);
        leftArmServo = new LeftActuatorServo(hardwareMap);
        leftActuator = new LeftActuator(hardwareMap);
        rightActuator = new RightActuator(hardwareMap);
    }


    public Action scoreSampleActionAuto() {
        return new SequentialAction(
                new ParallelAction(
                        arm.armScoreAction(),
                        wrist.wristVerticalAction()
                )
                , lift.liftUpAction()
                , wrist.wristFoldOutAction()
                , new SleepAction(PRE_DROP_SLEEP)
                , claw.clawOpenAction()
                , new SleepAction(POST_DROP_SLEEP)
                , wrist.wristVerticalAction()
                , new SleepAction(POST_DROP_SLEEP2)
        );
    }
    public Action comeDownActionAuto(){
        return new SequentialAction(
                wrist.wristVerticalAction(),
                new SleepAction(PRE_PICKUP_SLEEP),
                lift.liftOutPickUpAction(),
                wrist.wristFoldInAction(),
                new SleepAction(POST_PICKUP_SLEEP),
                arm.armPickupGroundSampleLiftOutAction()
        );
    }

    public Action comeDownSample3ActionAuto(){
        return new SequentialAction(
                wrist.wristVerticalAction(),
                new SleepAction(0.3),
                lift.liftOutPickUpAction(),
                new ParallelAction(
                        wrist.wristPickUpSample3Action(),
                        arm.armClearBarLiftOutAction()
                )
        );
    }

    public Action pickUpActionAuto(){
        return new SequentialAction(
                new SleepAction(0.4)
                ,claw.clawCloseAction(),
                new SleepAction(0.4)
        );
    }

    public Action resetAction() {
        return new SequentialAction(
                wrist.wristVerticalAction(),
                new SleepAction(0.5),
                new ParallelAction(
                        wrist.wristFoldInAction(),
                        lift.liftDownAction(),
                        arm.armResetAction()
                )
        );
    }

    public Action scoreSampleTeleOpAction(){
        return new SequentialAction(
                new ParallelAction(
                        arm.armScoreAction(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                wrist.wristVerticalAction()
                        ))
                , lift.liftUpAction()
                , wrist.wristFoldOutAction()
        );
    }

    public Action comeDownTeleopAction(){
        return new SequentialAction(
                wrist.wristVerticalAction(),
                new SleepAction(0.5),
                lift.liftOutPickUpAction(),
                new ParallelAction(
                        wrist.wristFoldInAction(),
                        arm.armClearBarLiftOutAction()
                )
        );
    }
}
