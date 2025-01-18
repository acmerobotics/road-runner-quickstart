package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.Constants;

public class Robot {

    public static MotorEx TopLeft;
    public static MotorEx TopRight;
    public static MotorEx BottomLeft;
    public static MotorEx BottomRight;
    public static DcMotor Arm1, Arm2;
    
    public static SimpleServo Claw;
    public static SimpleServo Claw2;
    public static SimpleServo Hanging;

    public static SimpleServo YServo;
    public static SimpleServo Drone;

    private static HardwareMap hwMapRobot;



    public static void init(HardwareMap hwMap, boolean drive) {

        hwMapRobot = hwMap;

        Arm1 = hwMap.get(DcMotor.class, Constants.Arm1ConfigName);
        Arm2 = hwMap.get(DcMotor.class, Constants.Arm2ConfigName);
        
        YServo = new SimpleServo(hwMapRobot, Constants.YServoConfigName, 0.0, 1.0);
        
        Claw = new SimpleServo(hwMapRobot, Constants.Claw1ConfigName, 0.0, 1.0);
        
        Claw2 = new SimpleServo(hwMapRobot, Constants.Claw2ConfigName, 0.0, 1.0);
        
        Drone = new SimpleServo(hwMapRobot, Constants.DroneConfigName, 0.0, 1.0);
        
        Hanging = new SimpleServo(hwMapRobot, Constants.HangingConfigName, 0.0, 1.0);

        if (drive){
            initDriveBase();
        }
    }
    private static void initDriveBase(){
        TopLeft = new MotorEx(hwMapRobot, "FL", Motor.GoBILDA.RPM_435);
        TopLeft.setInverted(true);

        TopRight = new MotorEx(hwMapRobot, "FR", Motor.GoBILDA.RPM_435);
        TopRight.setInverted(true);

        BottomLeft = new MotorEx(hwMapRobot, "BL", Motor.GoBILDA.RPM_435);
        BottomLeft.setInverted(true);

        BottomRight = new MotorEx(hwMapRobot, "BR", Motor.GoBILDA.RPM_435);
        BottomRight.setInverted(true);

    }

}