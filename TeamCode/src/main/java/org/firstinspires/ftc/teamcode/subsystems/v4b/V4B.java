package org.firstinspires.ftc.teamcode.subsystems.v4b;

import org.firstinspires.ftc.teamcode.util.StepperServo;

public class V4B {
    StepperServo servo1;
    StepperServo servo2;

    float target = 0;

    public V4B(StepperServo s1, StepperServo s2) {
        servo1 = s1;
        servo2 = s2;
    }

    public void runToPosition(float t) {
        servo1.setAngle(t);
        servo2.setAngle(t);

        target = t;
    }

    public float getPosition() {
        return target;
    }
}
