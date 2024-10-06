package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor flip, slide, intake;
    Servo claw, wrist;
    MecanumDrive drive;

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        flip = hardwareMap.dcMotor.get("flip");
        slide = hardwareMap.dcMotor.get("slide");
        intake = hardwareMap.dcMotor.get("intake");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");
    }
}
