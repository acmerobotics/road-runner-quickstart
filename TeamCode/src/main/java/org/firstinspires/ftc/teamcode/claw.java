package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class claw {
    private Servo clawServo;
    //variables to help set claw open and close positions
    private final double CLAMPED_POSITION = 0.2;
    private final double OPEN_POSITION = 1;

    //constructor
    public claw(Servo clawServo) {
        this.clawServo = clawServo;
    }

    //method for opening the claw
    public void clawOpen() {
        clawServo.setPosition(OPEN_POSITION);
    }

    //closing claw
    public void clawClose() {
        clawServo.setPosition(CLAMPED_POSITION);
    }
}
