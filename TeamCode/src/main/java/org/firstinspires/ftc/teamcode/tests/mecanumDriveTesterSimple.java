package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="mecanumDriveTesterSimple", group="Samples")

public class mecanumDriveTesterSimple extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public MecanumHardware mecanumHardware = new MecanumHardware();
    double deadzone = .3;
    double maxSpeed = 1;
    double x = 0;
    double y = 0;
    double rx = 0;

    public HardwareMap hwMap = null;



    @Override
    public void init() {
        // Save reference to Hardware map
        mecanumHardware.init(hardwareMap);
        mecanumHardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mecanumHardware.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }


    @Override
    public void loop() {

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

    @Override
    public void stop() {
    }

}