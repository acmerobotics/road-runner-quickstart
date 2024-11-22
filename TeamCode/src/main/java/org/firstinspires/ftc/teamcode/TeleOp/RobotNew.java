package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.Constants;

public class RobotNew {

    public MotorEx FrontLeft;
    public MotorEx FrontRight;
    public MotorEx BackLeft;
    public MotorEx BackRight;

    public MotorEx SlideLeft;
    public MotorEx SlideRight;
    public MotorGroup Slides;

    public MotorEx ExtensionRight;
    public MotorEx ExtensionLeft;
    public MotorGroup Extension;

    public SimpleServo HangingLowBar;
    public SimpleServo HangingHighBar;
    public SimpleServo FourBarLeft;
    public SimpleServo ClawGrip;
    public SimpleServo ClawRotation;

    private static HardwareMap hwMapRobot;


    public void init(HardwareMap hwMap) {

        hwMapRobot = hwMap;

        SlideLeft = new MotorEx(hwMap, Constants.SlideLeftName);
        SlideLeft.setRunMode(Motor.RunMode.RawPower);
        SlideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        SlideRight = new MotorEx(hwMap, Constants.SlideRightName);
        SlideRight.setRunMode(Motor.RunMode.RawPower);
        SlideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        Slides = new MotorGroup(SlideLeft, SlideRight);

        ExtensionRight = new MotorEx(hwMap, "ER");
        ExtensionRight.setRunMode(Motor.RunMode.RawPower);
        ExtensionRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        ExtensionLeft = new MotorEx(hwMap, "EL");
        ExtensionLeft.setRunMode(Motor.RunMode.RawPower);
        ExtensionLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        Extension = new MotorGroup(ExtensionLeft, ExtensionRight);

        FourBarLeft = new SimpleServo(hwMap, "VFBL", 0.0, 1.0);
        ClawGrip = new SimpleServo(hwMap, "CL", 0.0, 1.0);
        ClawRotation = new SimpleServo(hwMap, "CR", 0.0, 1.0);

//        YServo = new SimpleServo(hwMapRobot, Constants.YServoConfigName, 0.0, 1.0);
//        Claw = new SimpleServo(hwMapRobot, Constants.Claw1ConfigName, 0.0, 1.0);
//        HangingHighBar = new SimpleServo(hwMap, Constants.HangingHighName, 0.0, 1.0);
//        HangingLowBar = new SimpleServo(hwMap, Constants.HangingLowName, 0.0, 1.0);

        FrontLeft = new MotorEx(hwMap, "FL", Motor.GoBILDA.RPM_312);
        FrontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        FrontRight = new MotorEx(hwMap, "FR", Motor.GoBILDA.RPM_312);
        FrontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackLeft = new MotorEx(hwMap, "BL", Motor.GoBILDA.RPM_312);
        BackLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        BackRight = new MotorEx(hwMap, "BR", Motor.GoBILDA.RPM_312);
        BackRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}