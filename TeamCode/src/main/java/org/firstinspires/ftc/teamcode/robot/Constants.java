package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.math.Pose;


@Config
public class Constants {
    public enum Side {
        LEFT,
        RIGHT
    }

    public static int LEFTSIDE_REGION_X = 25;
    public static int LEFTSIDE_REGION_Y = 170;

    public static int RIGHTSIDE_REGION_X = 200;
    public static int RIGHTSIDE_REGION_Y = 165;

    public static int REGION_WIDTH = 75;
    public static int REGION_HEIGHT = 55;

    public static Pose yummypose = new Pose(0, 0, 0);
    public static Pose error = new Pose(0, 0, 0);
    public static Pose targetPose = new Pose(0, 0, 0);
    public static boolean reached = false;

    public static Side SIDE = Side.LEFT;
    public static boolean AUTO = false;
    public static boolean USING_IMU = true;

    public static boolean USE_WHEEL_FEEDFORWARD = false;

}
