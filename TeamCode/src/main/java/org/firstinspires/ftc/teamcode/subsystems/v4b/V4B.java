package org.firstinspires.ftc.teamcode.subsystems.v4b;

import org.firstinspires.ftc.teamcode.util.StepperServo;

public class V4B {
    StepperServo servo;

    float target = 0;

    public V4B(StepperServo s) {
        servo = s;
    }

    public void runToPosition(float t) {
        servo.setAngle(t);

        target = t;
    }

    public float getPosition() {
        return target;
    }
}
