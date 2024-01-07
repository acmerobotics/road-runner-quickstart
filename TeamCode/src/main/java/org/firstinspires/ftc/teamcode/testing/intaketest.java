package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="intaketest", group="Linear Opmode")
public class intaketest extends LinearOpMode
{
    @Override
    public void runOpMode(){
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        while (opModeIsActive()) {
            intake.setPower(gamepad1.left_stick_y);
        }
    }
}
