package org.firstinspires.ftc.teamcode.hardware.tidev2.pidftuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;


@TeleOp
public class ShoulderPIDF extends LinearOpMode {

    DcMotorEx shoulder_right;
    DcMotorEx shoulder_left;
    double currentVelocity;
    double maxVelocity = 0.0;

    private OpMode myOpMode;

    @Override
    public void runOpMode() throws InterruptedException {
        shoulder_right = hardwareMap.get(DcMotorEx.class, "shoulder_right");
        shoulder_left = hardwareMap.get(DcMotorEx.class, "shoulder_left");
        shoulder_right.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder_left.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {
            if (myOpMode.gamepad2.x) {
                shoulder_left.setPower(1);
                shoulder_right.setPower(1);
            } else if (myOpMode.gamepad2.a) {
                shoulder_left.setPower(-1);
                shoulder_right.setPower(-1);
            } else {
                shoulder_left.setPower(0);
                shoulder_right.setPower(0);
            }

            currentVelocity = (shoulder_left.getVelocity() + shoulder_right.getVelocity()) / 2;

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();


        }
    }
}