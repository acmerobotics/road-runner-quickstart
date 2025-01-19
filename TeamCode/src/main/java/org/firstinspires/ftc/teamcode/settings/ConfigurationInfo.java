package org.firstinspires.ftc.teamcode.settings;

import com.aimrobotics.aimlib.util.HardwareInterface;

public class ConfigurationInfo {

    //
    // v2 CONFIGURATION INFORMATION
    //

    // SERVOS
    public static HardwareInterface hand = new HardwareInterface("HND", true, 0);
    public static HardwareInterface leftElbow = new HardwareInterface("LELB", true, 0);
    public static HardwareInterface rightElbow = new HardwareInterface("RELB", true, 0);
    public static HardwareInterface rotator = new HardwareInterface("ROT", true, 0);
    public static HardwareInterface flexor = new HardwareInterface("FLX", true, 0);

    public static HardwareInterface leftSlide = new HardwareInterface("LS", true, 0);
    public static HardwareInterface rightSlide = new HardwareInterface("RS", true, 0);
    public static HardwareInterface pivot = new HardwareInterface("PVT", true, 0);



    //
    // v1 CONFIGURATION INFORMATION
    //


    // INTAKE BRISTLES
    public static HardwareInterface bristles = new HardwareInterface("BRI", true, 0);

    //INTAKE HINGE
    public static HardwareInterface leftHinge = new HardwareInterface("LH", false, 0);
    public static HardwareInterface rightHinge = new HardwareInterface("RH", false, 0);

    //INTAKE COLOR SENSORS
    public static HardwareInterface leftCS = new HardwareInterface("LCS", false, 0);
    public static HardwareInterface rightCS = new HardwareInterface("RCS", false, 0);

    public static HardwareInterface specimenGrabber = new HardwareInterface("SG", false, 0);

    // INTAKE SLIDES
    public static HardwareInterface leftIntakeSlide = new HardwareInterface("LIS", false, 2);
    public static HardwareInterface rightIntakeSlide = new HardwareInterface("RIS", true, 2);

    // INTAKE SLIDES PIVOT
    public static HardwareInterface leftIntakeSlidePivot = new HardwareInterface("LISP", false, 2);
    public static HardwareInterface rightIntakeSlidePivot = new HardwareInterface("RISP", true, 2);

    // OUTAKE SLIDES
    public static HardwareInterface leftOuttakeSlide = new HardwareInterface("LOS", false, 2);
    public static HardwareInterface rightOuttakeSlide = new HardwareInterface("ROS", true, 2);

    // OUTAKE ARM HINGES
    public static HardwareInterface leftArmHinge = new HardwareInterface("LOAH", true, 0);
    public static HardwareInterface rightArmHinge = new HardwareInterface("ROAH", true, 0);

    // OUTAKE BUCKET HINGE
    public static HardwareInterface bucketHinge = new HardwareInterface("BH", false, 0);

    // DRIVEBASE
    public static HardwareInterface leftFront = new HardwareInterface("FLD", true, 1);
    public static HardwareInterface rightFront = new HardwareInterface("FRD", false, 1);
    public static HardwareInterface leftBack = new HardwareInterface("BLD", true, 0);
    public static HardwareInterface rightBack = new HardwareInterface("BRD", false, 0);

    // CAMERA
    public static HardwareInterface camera = new HardwareInterface("Ray", true, 0);

    // IMU
    public static HardwareInterface imu = new HardwareInterface("imu", false, 0);
}
