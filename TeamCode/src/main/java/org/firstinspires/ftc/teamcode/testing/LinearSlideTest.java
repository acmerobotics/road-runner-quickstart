package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Viper slide test", group="Linear Opmode")

public class LinearSlideTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor viperSlideLeft = hardwareMap.get(DcMotor .class, "viper_slide_left");
        viperSlideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotor viperSlideRight = hardwareMap.get(DcMotor.class, "viper_slide_right");
        viperSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            viperSlideLeft.setPower(gamepad1.left_trigger);
            viperSlideRight.setPower(gamepad1.right_trigger);
        }
    }
}