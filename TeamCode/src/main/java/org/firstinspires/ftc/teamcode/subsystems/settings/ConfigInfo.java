package org.firstinspires.ftc.teamcode.subsystems.settings;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.HardwareInterface;

public class ConfigInfo {

    // ARM
    public static HardwareInterface leftArm = new HardwareInterface("leftArm", false, 0);
    public static HardwareInterface rightArm = new HardwareInterface("rightArm", false, 0);

    // CLAW
    public static HardwareInterface rotator = new HardwareInterface("rotator", false, 0);
    public static HardwareInterface leftProng = new HardwareInterface("leftProng", false, 0);
    public static HardwareInterface rightProng = new HardwareInterface("rightProng", false, 0);

    // DRIVEBASE
    public static HardwareInterface leftFront = new HardwareInterface("fld", false, 0);
    public static HardwareInterface rightFront = new HardwareInterface("frd", false, 0);
    public static HardwareInterface leftBack = new HardwareInterface("bld", false, 0);
    public static HardwareInterface rightBack = new HardwareInterface("brd", false, 0);

    // INTAKE
    public static HardwareInterface intake = new HardwareInterface("intake", false, 0);

    // PAL
    public static HardwareInterface releaser = new HardwareInterface("releaser", false, 0);
    public static HardwareInterface fire = new HardwareInterface("fire", false, 0);

    // SLIDES
    public static HardwareInterface leftSlide = new HardwareInterface("leftSlide", false, 0);
    public static HardwareInterface rightSlide = new HardwareInterface("rightSlide", false, 0);

    // CAMERA
    public static HardwareInterface camera = new HardwareInterface("Ray", true, 0);
}
