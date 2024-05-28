package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class motorTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("motor1");
        DcMotor motor6 = hardwareMap.dcMotor.get("motor6");
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(0.4);
                motor6.setPower(0.4);
                TimeUnit.MILLISECONDS.sleep(350);
            } else if (gamepad1.b) {
                motor.setPower(0);
                motor6.setPower(0.1);
                TimeUnit.MILLISECONDS.sleep(350);
            }
        }
    }
}