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

    public Robotv2(HardwareMap hardwareMap, Pose2d startPose){
        drive = new MecanumDrive(hardwareMap, startPose);
        arm = new Armv2(hardwareMap);
        wrist = new Wristv2(hardwareMap);
        lift = new Liftv2(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    public Action scoreSampleAction(){
        return new SequentialAction(
                new ParallelAction(
                        arm.armScoreAction(),
                        lift.liftUpAction(),
                        wrist.wristVerticalAction()),
                wrist.wristFoldOutAction(),
                new SleepAction(2),
                claw.clawOpenAction(),
                new SleepAction(.2)
        );
    }

    public Action comeDownAction(){
        return new SequentialAction(
                wrist.wristVerticalAction(),
                new SleepAction(0.5),
                lift.liftDownAction(),
                new ParallelAction(
                        wrist.wristFoldInAction(),
                        arm.armPickupGroundSampleAction()
                )
        );
    }

}
