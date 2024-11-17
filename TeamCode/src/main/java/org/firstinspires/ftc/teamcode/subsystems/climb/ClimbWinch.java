package org.firstinspires.ftc.teamcode.subsystems.climb;

import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class ClimbWinch {
    public StepperServo servo1;
    StepperServo servo2;

    float target = 0;

    public static int PRIMED_POS = 0;
    public static int EXTENDED_POS = 120;
    public static int CLIMB_POS = 220;

    public ClimbWinch(StepperServo s1, StepperServo s2) {
        servo1 = s1;
        servo2 = s2;
    }

    public void runToPosition(float t) {
        servo1.setAngle(t);
        servo2.setAngle(t);

        target = t;
    }

    public void prime() {
        runToPosition(PRIMED_POS);
    }

    public void up() {
        if (target != EXTENDED_POS) {
            runToPosition(EXTENDED_POS);
        } else {
            // disengage winch if pressed by accident
            prime();
        }

    }

    public void climb() {
        runToPosition(CLIMB_POS);
    }

    public float getPosition() {
        return target;
    }
}
