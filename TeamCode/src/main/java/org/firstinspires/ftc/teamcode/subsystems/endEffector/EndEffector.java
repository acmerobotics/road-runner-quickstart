package org.firstinspires.ftc.teamcode.subsystems.endEffector;

import org.firstinspires.ftc.teamcode.util.ContinuousServo;

public class EndEffector {
    ContinuousServo servo1;
    ContinuousServo servo2;

    public EndEffector(ContinuousServo s1, ContinuousServo s2) {
        servo1 = s1;
        servo2 = s2;
    }

    public void setPower(float p) {
        servo1.servo.setPower(p);
        servo2.servo.setPower(p);
    }
}
