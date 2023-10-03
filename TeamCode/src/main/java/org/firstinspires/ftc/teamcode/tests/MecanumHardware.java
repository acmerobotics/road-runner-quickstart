package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumHardware {

    public HardwareMap hwMap = null;
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    /* Constructor */
    public MecanumHardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "front_left_drive");
        rightFront = hwMap.get(DcMotorEx.class, "front_right_drive");
        leftRear = hwMap.get(DcMotorEx.class, "back_left_drive");
        rightRear = hwMap.get(DcMotorEx.class, "back_right_drive");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}

