package org.firstinspires.ftc.teamcode.hardware.tidev2.pidftuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class ViperPIDF extends LinearOpMode {

    DcMotorEx viper;
    double currentVelocity;
    double maxVelocity = 0.0;

    private OpMode myOpMode;

    @Override
    public void runOpMode() throws InterruptedException {
        viper = hardwareMap.get(DcMotorEx.class, "viper_slide");
        viper.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();
        while (opModeIsActive()) {
            if (myOpMode.gamepad2.x) {
                viper.setPower(1);
            } else if (myOpMode.gamepad2.a) {
                viper.setPower(-1);
            } else {
                viper.setPower(0);
            }

            currentVelocity = viper.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();


        }
    }
}