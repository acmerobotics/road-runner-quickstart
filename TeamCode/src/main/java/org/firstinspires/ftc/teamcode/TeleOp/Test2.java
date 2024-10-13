package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test2")
public class Test2 extends LinearOpMode {
    static GamepadEx gamepadone;
    static GamepadEx gamepadtwo;
    static MotorEx SlideLeft;

    public void HardwareStart(){

        SlideLeft = new MotorEx(hardwareMap, "SL", Motor.GoBILDA.RPM_312);
        SlideLeft.setRunMode(Motor.RunMode.RawPower);
        SlideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        SlideLeft.setRunMode(Motor.RunMode.PositionControl);

        gamepadone = new GamepadEx(gamepad1);
        gamepadtwo = new GamepadEx(gamepad2);
    }

    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.dpad_up){
                SlideLeft.set(-1);
            } else if(gamepad2.dpad_down){
                SlideLeft.set(1);
            } else {
                SlideLeft.set(-0.009375);
            }

//            SlideLeft.set(1);
//            if (gamepad2.dpad_up){
//                SlideLeft.setTargetPosition(SlideLeft.getCurrentPosition() + 10);
//            } else if (gamepad2.dpad_down){
//                SlideLeft.setTargetPosition(SlideLeft.getCurrentPosition() - 10);
//            } else {
//                SlideLeft.setTargetPosition(SlideLeft.getCurrentPosition());
//            }


        }
    }
}
