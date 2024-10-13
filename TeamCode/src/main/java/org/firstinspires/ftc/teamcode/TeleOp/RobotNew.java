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

    public SimpleServo Extension;
    public SimpleServo HangingLowBar;
    public SimpleServo HangingHighBar;
    public SimpleServo Claw;
    public SimpleServo YServo;

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

        YServo = new SimpleServo(hwMapRobot, Constants.YServoConfigName, 0.0, 1.0);
        Claw = new SimpleServo(hwMapRobot, Constants.Claw1ConfigName, 0.0, 1.0);
        Extension = new SimpleServo(hwMap, Constants.ExtensionName, 0.0, 1.0);
        HangingHighBar = new SimpleServo(hwMap, Constants.HangingHighName, 0.0, 1.0);
        HangingLowBar = new SimpleServo(hwMap, Constants.HangingLowName, 0.0, 1.0);

        FrontLeft = new MotorEx(hwMap, "FL", Motor.GoBILDA.RPM_312);

        FrontRight = new MotorEx(hwMap, "FR", Motor.GoBILDA.RPM_312);

        BackLeft = new MotorEx(hwMap, "BL", Motor.GoBILDA.RPM_312);
        FrontRight.setInverted(true);

        BackRight = new MotorEx(hwMap, "BR", Motor.GoBILDA.RPM_312);
    }
}