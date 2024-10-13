package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slides Test")
public class SlidesTest extends LinearOpMode {
    static MecanumDrive drive;
    static GamepadEx gamepadone;
    static GamepadEx gamepadtwo;
    static MotorEx FrontLeft;
    static MotorEx FrontRight;
    static MotorEx BackLeft;
    static MotorEx BackRight;
    static MotorEx SlideLeft;
    static MotorEx SlideRight;
    static SimpleServo extension;

    public void HardwareStart(){
        FrontLeft = new MotorEx(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        FrontRight = new MotorEx(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        BackLeft = new MotorEx(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        FrontRight.setInverted(true);
        BackRight = new MotorEx(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        drive = new MecanumDrive(FrontLeft, FrontRight, BackLeft, BackRight);

        SlideLeft = new MotorEx(hardwareMap, "SL", Motor.GoBILDA.RPM_312);
        SlideLeft.setRunMode(Motor.RunMode.RawPower);
        SlideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        SlideRight = new MotorEx(hardwareMap, "SR", Motor.GoBILDA.RPM_312);
//        SlideRight.setRunMode(Motor.RunMode.VelocityControl);

        extension = new SimpleServo(hardwareMap, "ES", 0.0, 1.0);

        gamepadone = new GamepadEx(gamepad1);
        gamepadtwo = new GamepadEx(gamepad2);
    }

    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

        while(opModeIsActive()){
            drive.driveRobotCentric(
                    gamepadone.getLeftX(),
                    gamepadone.getLeftY(),
                    gamepadone.getRightX(),
                    true
            );

            if(gamepad2.a){
                extension.setPosition(0);
            }

            if(gamepad2.b){
                extension.setPosition(1);
            }

            if(gamepad2.dpad_up){
                SlideLeft.set(1);
            } else {
                SlideLeft.set(0);
            }

            if(gamepad2.dpad_down){
                SlideLeft.set(-1);
            } else {
                SlideLeft.set(0);
            }
        }
    }
}
