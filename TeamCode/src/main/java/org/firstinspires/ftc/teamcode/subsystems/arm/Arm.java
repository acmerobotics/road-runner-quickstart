package org.firstinspires.ftc.teamcode.subsystems.arm;

import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Arm {
    StepperServo servo;
    StepperServo wrist;

    float target = 0;

    public Arm(StepperServo s1, StepperServo s2) {
        servo = s1;
        wrist = s2;
    }

    public void runToPosition(float t) {
        servo.setAngle(t);

        target = t;
    }

    public void runToPreset(Levels level) {
        if (level == Levels.INTAKE) {
            runToPosition(1);
        }
    }

    public float getPosition() {
        return target;
    }

    public void setWristAngle(float angle) {
        wrist.setAngle(angle);
    }

    public void runToWristPreset(Levels level) {
        if (level == Levels.INTAKE_INTERMEDIATE) {
            setWristAngle(10);
        }
    }
}
