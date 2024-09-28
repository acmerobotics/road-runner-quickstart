package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Constants {
    public static final Pose2d RED_RIGHT_START = new Pose2d(12, -62, Math.toRadians(90));
    public static final Pose2d RED_LEFT_START = new Pose2d(-36, -62, Math.toRadians(90));
    public static final Pose2d BLUE_RIGHT_START = new Pose2d(-36, 62, Math.toRadians(270));
    public static final Pose2d BLUE_LEFT_START = new Pose2d(12, 62, Math.toRadians(270));
    public static final int ArmUpTicks = 1800;
    public static final int ArmUpTicksAuto = 1890;
    public static final double armSpeedUp = 0.6;
    public static final double armSpeedDown = 0.2;
    public static final String Arm1ConfigName = "A1";
    public static final String Arm2ConfigName = "A2";
    public static final int armIncrement = 60;
    public static final double YServoUp = 1;
    public static final double YServoUpAuto = 1;
    public static final double YServoDown = 0.52;
    public static final double YServoDownAuto = 0.57;
    public static final double YServoDownFull = 0.4;
    public static final String YServoConfigName = "YS";
    public static final double Claw2Closed = 1;
    public static final double ClawClosed = 0.3;
    public static final double ClawOpen = 0.8;
    public static final double Claw2Open = 0.3;
    public static final String Claw1ConfigName = "CLAW";
    public static final String Claw2ConfigName = "CLAW2";
    public static final double DroneReset = 1;
    public static final double DroneFly = 0;
    public static final String DroneConfigName = "DRONE";
    public static final double HangingDown = 0;
    public static final double HangingMiddle = 0.38;
    public static final double HangingUp = 1;
    public static final String HangingConfigName = "HANGING";

}
