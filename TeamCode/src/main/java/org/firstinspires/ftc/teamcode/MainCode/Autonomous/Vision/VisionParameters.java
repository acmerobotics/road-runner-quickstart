package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionParameters {
    public static int readTime = 500;
    public static int resX = 320;
    public static int resY = 176;

    public static int blueHueMax = 20;
    public static int blueHueMin = 0;
    public static int blueSatMax = 255;
    public static int blueSatMin = 95;
    public static int blueValMax = 215;
    public static int blueValMin = 0;

    public static int redHueMax = 140;
    public static int redHueMin = 105;
    public static int redSatMax = 255;
    public static int redSatMin = 55;
    public static int redValMax = 255;
    public static int redValMin = 10;

    public static int leftStartX = 0;
    public static int leftStartY = 0;
    public static int leftEndX = resX;
    public static int leftEndY = resY;

    public static int middleStartX = 0;
    public static int middleStartY = 0;
    public static int middleEndX = resX;
    public static int middleEndY = resY;

    public static int rightStartX = 0;
    public static int rightStartY = 0;
    public static int rightEndX = resX;
    public static int rightEndY = resY;
}
