package org.firstinspires.ftc.teamcode.mechanisms.robotv2;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robotv2 {
    public MecanumDrive drive;
    public Armv2 arm;
    public Wristv2 wrist;
    public Liftv2 lift;
    public Claw claw;
    public RightActuatorServo rightArmServo;
    public LeftActuatorServo leftArmServo;

    public LeftActuator leftActuator;
    public RightActuator rightActuator;

    public Robotv2(HardwareMap hardwareMap, Pose2d startPose){
        drive = new MecanumDrive(hardwareMap, startPose);
        arm = new Armv2(hardwareMap);
        wrist = new Wristv2(hardwareMap);
        lift = new Liftv2(hardwareMap);
        claw = new Claw(hardwareMap);

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
                        new SequentialAction(
                                new SleepAction(0.5),
                                wrist.wristVerticalAction()
                        ))
                , lift.liftUpAction()
                , wrist.wristFoldOutAction()
                , new SleepAction(0.5)
                , claw.clawOpenAction()
                , new SleepAction(.2)
        );
    }
    public Action comeDownActionAuto(){
        return new SequentialAction(
                wrist.wristVerticalAction(),
                new SleepAction(0.3),
                lift.liftOutPickUpAction(),
                new ParallelAction(
                        wrist.wristFoldInAction(),
                        arm.armPickupGroundSampleLiftOutAction()
                )
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
