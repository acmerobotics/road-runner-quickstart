package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class ChassisController {
    DcMotorEx leftFront, rightFront, leftRear, rightRear;
    public static double speed = 0.6;
    public ChassisController(HardwareMap map)
    {
        leftFront = map.get(DcMotorEx.class, "m1");
        rightFront = map.get(DcMotorEx.class, "m3");
        leftRear = map.get(DcMotorEx.class, "m0");
        rightRear = map.get(DcMotorEx.class, "m2");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(Gamepad g)
    {
        leftFront.setPower((-g.left_stick_y + g.left_stick_x + g.right_stick_x) * speed);
        rightFront.setPower((-g.left_stick_y - g.left_stick_x - g.right_stick_x) * speed);
        leftRear.setPower((-g.left_stick_y - g.left_stick_x + g.right_stick_x) * speed);
        rightRear.setPower((-g.left_stick_y + g.left_stick_x - g.right_stick_x) * speed);

    }

}
