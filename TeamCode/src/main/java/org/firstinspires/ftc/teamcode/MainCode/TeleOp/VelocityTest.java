package org.firstinspires.ftc.teamcode.MainCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class VelocityTest extends LinearOpMode {

    DcMotorEx motor;

    double currentVelocity;

    double maxVelocity = 0.0;


    @Override

    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "outtake_elbow");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while (opModeIsActive()) {
            motor.setPower(-1);
            currentVelocity = motor.getVelocity();



            if (currentVelocity > maxVelocity) {

                maxVelocity = currentVelocity;

            }



            telemetry.addData("current velocity", currentVelocity);

            telemetry.addData("maximum velocity", maxVelocity);

            telemetry.update();

        }

    }

}
