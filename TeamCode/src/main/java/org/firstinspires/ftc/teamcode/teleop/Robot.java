package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor flip, slide;
    Servo claw, wrist, intakeLeft, intakeRight;
    MecanumDrive drive;

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        flip = hardwareMap.dcMotor.get("flip");
        slide = hardwareMap.dcMotor.get("slide");
        intakeLeft = hardwareMap.servo.get("intakeLeft");
        intakeRight = hardwareMap.servo.get("intakeRight");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Action pidfLoopFlip(double target) {
        pidfLoop loop = new pidfLoop();
        return loop.getPivotLoop(flip, target);
    }

    public Action pidfLoopSlides(double target) {
        pidfLoop loop = new pidfLoop();
        return loop.getSlidesLoop(slide, target);
    }

    public void setTelemToDashboard(Telemetry telem) {
        telem = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }
}
