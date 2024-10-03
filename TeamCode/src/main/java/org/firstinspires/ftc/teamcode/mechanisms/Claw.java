package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo clawLeftServo;
    Servo clawRightServo;
    Servo clawServo;

    public Claw(HardwareMap HWMap) {
        clawLeftServo = HWMap.get(Servo.class, "clawLeftServo");
        clawRightServo = HWMap.get(Servo.class, "clawLeftServo");
        clawServo = HWMap.get(Servo.class, "clawServo");
    }

    public void flip() {
        clawLeftServo.setPosition(1);
        clawRightServo.setPosition(1);
    }

    public void flop() {
        clawLeftServo.setPosition(0);
        clawRightServo.setPosition(0);
    }

    public void close() {
        clawServo.setPosition(1);
    }

    public void open() {
        clawServo.setPosition(0);
    }
}
