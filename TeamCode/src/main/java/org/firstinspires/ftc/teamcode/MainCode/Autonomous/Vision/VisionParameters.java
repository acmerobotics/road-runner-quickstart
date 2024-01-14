package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionParameters {
    public static int readTime = 500;
    public static int redConfidance = 0;
    public static int blueConfidance = 0;

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

    public static int closeStartX = 0;
    public static int closeStartY = 0;
    public static int closeEndX = 320;
    public static int closeEndY = 240;

    public static int farStartX = 0;
    public static int farStartY = 0;
    public static int farEndX = 320;
    public static int farEndY = 240;
}
