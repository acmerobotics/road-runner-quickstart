package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class MegaControl extends OpMode {

    Servo intake; // Intake declaration
    public String intakeName = "intake";

    DcMotor arm;
    DcMotor arm2;
    public String armName = "arm";
    double extend = 1;
    double reduce = 0;

    Servo clawHookLeft;
    Servo clawHookRight;

    Servo clawLifter;
    Servo clawLifter2;

    Servo clawRotator;


    public String clawName = "clawDustpan";


    //private ElapsedTime runtime = new ElapsedTime();
    public MecanumHardware mecanumHardware = new MecanumHardware();

    double deadzone = .3;
    double maxSpeed = .8;
    double x = 0;
    double y = 0;
    double rx = 0;

    public HardwareMap hwMap = null;

    @Override
    public void init() {
        intake = hwMap.get(Servo.class, intakeName);
        arm = hwMap.get(DcMotor.class, armName);
        arm2 = hwMap.get(DcMotor.class, armName);
        clawHookLeft = hwMap.get(Servo.class, clawName);
        clawHookRight= hwMap.get(Servo.class, clawName);
        clawLifter = hwMap.get(Servo.class, clawName);
        clawLifter2 = hwMap.get(Servo.class, clawName);
        clawRotator = hwMap.get(Servo.class, clawName);
        MecanumHardware mecanumHardware = new MecanumHardware();

        mecanumHardware.init(hardwareMap);
        mecanumHardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        //INTAKE
        if (gamepad1.a) {
            intake.setPosition(1);
        } else if (gamepad1.b) {
            intake.setPosition(-1);
        } else {
            intake.setPosition(0);
        }

        //ARMs
        if (gamepad1.a) {
            arm.setPower(1);
            arm2.setPower(1);
        } else if (gamepad1.b) {
            arm.setPower(-1);
            arm2.setPower(-1);
        } else
            arm.setPower(0);
            arm2.setPower(0);

        //CLAWHOOKS

        //LEFT
        if (gamepad1.a) {
            clawHookLeft.setPosition(1);
        } else if (gamepad1.b) {
            clawHookLeft.setPosition(1);
        } else {
            clawHookLeft.setPosition(1);
        }

        //RIGHT
        if (gamepad1.a) {
            clawHookRight.setPosition(1);
        } else if (gamepad1.b) {
            clawHookRight.setPosition(1);
        } else {
            clawHookRight.setPosition(1);
        }

        //CLAW LIFTER
        if (gamepad1.a) {
            clawLifter.setPosition(1);
            clawLifter2.setPosition(1);
        } else if (gamepad1.b) {
            clawLifter.setPosition(-1);
            clawLifter2.setPosition(-1);

        } else {
            clawLifter.setPosition(0);
            clawLifter2.setPosition(0);
        }


        //CLAW ROTATOR
        if (gamepad1.a) {
            clawRotator.setPosition(1);
        } else if (gamepad1.b) {
            clawRotator.setPosition(-1);
        } else {
            clawRotator.setPosition(0);
        }



        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;


        if (Math.abs(-gamepad1.left_stick_y) > deadzone) {
            x = -gamepad1.left_stick_y * .7;
        } else {
            x = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) > deadzone) {
            y = gamepad1.left_stick_x * .65;
        } else {
            y = 0;
        }
        if (Math.abs(gamepad1.right_stick_x) > deadzone) {
            rx = gamepad1.right_stick_x * .85;
        } else {
            rx = 0;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x - rx) / denominator;
        frontRightPower = (y - x + rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        mecanumHardware.leftFront.setPower(frontLeftPower * maxSpeed);
        mecanumHardware.leftRear.setPower(backLeftPower * maxSpeed);
        mecanumHardware.rightFront.setPower(frontRightPower * maxSpeed);
        mecanumHardware.rightRear.setPower(backRightPower * maxSpeed);



    }

}

