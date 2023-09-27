package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;


public class Arm extends Mechanism {
    Servo arm;
    public String armName = "arm";
    public String arm2Name = "arm2";
    int extend = 1;
    int reduce = 0;

    @Override
    public void init(HardwareMap hwMap) {
        arm = hwMap.get(Servo.class, armName);
    }

    public void extend() {
        arm.setPosition(extend);
    }

    public void rescind() {
        arm.setPosition(reduce);
    }
}
