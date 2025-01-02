package org.firstinspires.ftc.teamcode.mechanisms.robotv2;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robotv2 {
    public MecanumDrive drive;
    public Armv2 arm;
    public Wristv2 wrist;
    public Liftv2 lift;
    public Claw claw;

    Robotv2(HardwareMap hardwareMap, Pose2d startPose){
        drive = new MecanumDrive(hardwareMap, startPose);
        arm = new Armv2(hardwareMap);
        wrist = new Wristv2(hardwareMap);
        lift = new Liftv2(hardwareMap);
        claw = new Claw(hardwareMap);
    }
}
