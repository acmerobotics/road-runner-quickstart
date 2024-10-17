package org.firstinspires.ftc.teamcode.club;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Minibots", group="ZZZ_CLUB")
public class Minibots extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private TouchSensor killSwitch;

    @Override
    public void init() {
        // Initialize motors
        leftMotor = hardwareMap.dcMotor.get("left");  // Replace "left_motor" with the actual name in your robot configuration
        rightMotor = hardwareMap.dcMotor.get("right");  // Replace "right_motor" with the actual name in your robot configuration

        // Set motor directions
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE); //TODO SWITCH TO REVERSE FOR THE 4 WHEEL

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        killSwitch = hardwareMap.touchSensor.get("kill");
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y * 0.65;
        double turn = gamepad1.right_stick_x * 0.4;

        if (gamepad1.b) {
            y = (gamepad1.right_trigger + -gamepad1.left_trigger) * 0.8;
        }
        // Tank drive control
        double leftPower = y + turn;  // Adjust if needed
        double rightPower = y - turn;  // Adjust if needed

        // Set motor powers
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (killSwitch.isPressed()) {
            requestOpModeStop();
        }

        telemetry.addData("Button is pressed:", killSwitch.isPressed());
    }
}
