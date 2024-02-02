package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name = "Get nums", group = "comp")
public class getNumbers extends OpMode {


    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor slideLeft;
    DcMotor slideRight;

    DcMotor wristMotor;
    Servo gripServoF;
    Servo gripServoB;
    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        wristMotor = hardwareMap.dcMotor.get("wristMotor");
        gripServoF = hardwareMap.servo.get("gripServoF");
        gripServoB = hardwareMap.servo.get("gripServoB");

        gripServoB.setPosition(0);
        gripServoF.setPosition(0);
        //reverse  the
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Servo Position", gripServoB.getPosition());

    }

    @Override
    public void loop(){


        if (gamepad2.circle) {
            gripServoB.setPosition(1);
            gripServoF.setPosition(1);
            telemetry.addData("Servo PositionBWhile", gripServoB.getPosition());
            telemetry.addData("Servo PositionBWhile", gripServoB.getPosition()*300);

        } else {
            gripServoB.setPosition(0);
            gripServoB.setPosition(0);

        }
        telemetry.addData("Servo PositionB", gripServoB.getPosition());
        telemetry.addData("Servo PositionB", gripServoB.getPosition()*300);


        telemetry.addData("leftslide endcoderPOS", slideLeft.getCurrentPosition());
        telemetry.addData("right endcoderPOS", slideRight.getCurrentPosition());


    }
}
