package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="DcMotorArmTest", group="Tests")
public class DcMotorArmTest extends OpMode {
    // A teleOp to test the arm

    DcMotorEx arm1;
    DcMotorEx arm2;

    @Override
    public void init() {
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
        arm1.setDirection(DcMotorEx.Direction.REVERSE);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            arm1.setPower(1);
            arm2.setPower(1);
        } else if (gamepad1.b) {
            arm1.setPower(-1);
            arm2.setPower(-1);
        } else {
            arm1.setPower(0);
            arm2.setPower(0);
        }
    }
}
