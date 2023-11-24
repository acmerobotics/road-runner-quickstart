package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Controller;

//@Disabled

/** this is What WORKS **/


@TeleOp(name = "THIS IS WHAT WORKS - Manual Op")
public class ManualOp extends OpMode {

    //Here we are creating all the parts what we will manipulate
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor slideLeft;
    DcMotor slideRight;
    DcMotor slideTop;
    Servo gripServo;




    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideTop = hardwareMap.dcMotor.get("intakeMotor");
        gripServo = hardwareMap.servo.get("gripServo");

        gripServo.setPosition(0.5);

        //reverse  the
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.left_trigger - gamepad1.right_trigger;

        leftFront.setPower(-(y + x - rx));
        leftRear.setPower(-(y - x + rx));
        rightFront.setPower((y - x - rx));
        rightRear.setPower((y + x + rx));

        double vert = -gamepad2.left_stick_y;
        double top = gamepad2.right_stick_x;

        slideLeft.setPower(-vert);
        slideRight.setPower(vert);
        slideTop.setPower(top);

        double servoSpeed = ((gamepad2.right_trigger - gamepad2.left_trigger) + 1) / 2;

        telemetry.addData("servo speed: ", servoSpeed);
        telemetry.update();


        //These are the breaks
        //the way these are called will be different in the classes with the helpers

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        gripServo.setPosition(servoSpeed);

    }
}
