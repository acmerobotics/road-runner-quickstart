package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RunToPositionMotorUtil;


@TeleOp(name="MegaControl", group="Samples")
public class MegaControl extends OpMode {

    RunToPositionMotorUtil motorUtil = new RunToPositionMotorUtil();

    CRServo intake; // Intake declaration
    public String intakeName = "intake";

    CRServo leftClawPositioner;
    CRServo rightClawPositioner;
    public String leftClawPositionerName = "leftClawPositioner";
    public String rightClawPositionerName = "rightClawPositioner";

    DcMotorEx leftArm;
    DcMotorEx rightArm;
    public String leftArmName = "leftArm";
    public String rightArmName = "rightArm";

    public MecanumHardware mecanumHardware = new MecanumHardware();

    double deadzone = .3;
    double maxSpeed = .8;
    double x = 0;
    double y = 0;
    double rx = 0;



    int lastLeftPos = 0;
    int lastRightPos = 0;


    @Override
    public void init() {
        intake = hardwareMap.get(CRServo.class, intakeName);
        leftClawPositioner = hardwareMap.get(CRServo.class, leftClawPositionerName);
        rightClawPositioner = hardwareMap.get(CRServo.class, rightClawPositionerName);

        rightClawPositioner.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm = hardwareMap.get(DcMotorEx.class, leftArmName);
        rightArm = hardwareMap.get(DcMotorEx.class, rightArmName);

        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setTargetPositionTolerance(1);
        rightArm.setTargetPositionTolerance(1);
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
        if (gamepad1.x) {
            intake.setPower(1);
        } else if (gamepad1.b) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //ARM Code to keep the arm in place when there is no power applied to it.
        if (gamepad1.y) {
            leftArm.setPower(1);
            rightArm.setPower(1);
            lastLeftPos = leftArm.getCurrentPosition();
            lastRightPos = rightArm.getCurrentPosition();
        } else if (gamepad1.a) {
            leftArm.setPower(-1);
            rightArm.setPower(-1);
            lastLeftPos = leftArm.getCurrentPosition();
            lastRightPos = rightArm.getCurrentPosition();
        } else {
            motorUtil.motorToPosition(leftArm, 1, lastLeftPos);
            motorUtil.motorToPosition(rightArm, 1, lastRightPos);
        }

        //CLAW LIFTER
        if (gamepad1.right_bumper) {
            leftClawPositioner.setPower(1);
            rightClawPositioner.setPower(1);
        } else if (gamepad1.left_bumper) {
            leftClawPositioner.setPower(-1);
            rightClawPositioner.setPower(-1);
        } else {
            leftClawPositioner.setPower(0);
            rightClawPositioner.setPower(0);
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

