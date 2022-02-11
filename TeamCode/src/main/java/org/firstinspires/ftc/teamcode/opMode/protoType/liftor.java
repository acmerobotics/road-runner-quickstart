package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class liftor extends LinearOpMode {
    public boolean formerDpadL = false;
    public boolean formerDpadR = false;
    public DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("lift");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("encoder: ", motor.getCurrentPosition());
            telemetry.update();
            if(gamepad1.dpad_left && motor.getCurrentPosition() > -2800) {
                motor.setPower(0.375);
            }
            if(!gamepad1.dpad_left && gamepad1.dpad_right && motor.getCurrentPosition() < 2800)  {
                motor.setPower(-0.375);
            }
            if(gamepad1.a || motor.getCurrentPosition() > 2500 && !gamepad2.dpad_left|| motor.getCurrentPosition() < -2500 && !gamepad2.dpad_right){
                motor.setPower(0);
            }

        }
    }
}
