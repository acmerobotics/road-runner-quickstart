package org.firstinspires.ftc.teamcode.subsystems.settings;

import com.aimrobotics.aimlib.util.HardwareInterface;

public class ConfigurationInfo {

    // INTAKE BRISTLES
    public static HardwareInterface bristles = new HardwareInterface("BRI", true, 0);

    //INTAKE HINGE
    public static HardwareInterface leftHinge = new HardwareInterface("LH", false, 0);
    public static HardwareInterface rightHinge = new HardwareInterface("RH", false, 0);

    // INTAKE SLIDES
    public static HardwareInterface leftIntakeSlide = new HardwareInterface("LIS", false, 2);
    public static HardwareInterface rightIntakeSlide = new HardwareInterface("RIS", true, 2);

    // INTAKE SLIDES PIVOT
    public static HardwareInterface leftIntakeSlidePivot = new HardwareInterface("LISP", false, 2);
    public static HardwareInterface rightIntakeSlidePivot = new HardwareInterface("RISP", true, 2);

    // OUTAKE SLIDES
    public static HardwareInterface leftOutakeSlide = new HardwareInterface("LOS", false, 2);
    public static HardwareInterface rightOutakeSlide = new HardwareInterface("ROS", true, 2);

    // OUTAKE ARM HINGES
    public static HardwareInterface leftArmHinge = new HardwareInterface("OLAH", true, 0);
    public static HardwareInterface rightArmHinge = new HardwareInterface("ORAH", true, 0);

    // OUTAKE BUCKET HINGES
    public static HardwareInterface leftBucketHinge = new HardwareInterface("OLBH", false, 0);
    public static HardwareInterface rightBucketHinge = new HardwareInterface("ORBH", false, 0);

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
