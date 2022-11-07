package org.firstinspires.ftc.teamcode.claw;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo clawServo;
    //variables to help set claw open and close positions
    private static final double CLAMPED_POSITION = 0.2;
    private static final double OPEN_POSITION = .7;

    private boolean clawState = true;

    //constructor
    public Claw(final Servo clawServo) {
        this.clawServo = clawServo;
    }

    //method for opening the claw
    public void clawOpen() {
        clawServo.setPosition(OPEN_POSITION);
        this.clawState = true;
    }

    //closing claw
    public void clawClose() {
        clawServo.setPosition(CLAMPED_POSITION);
        this.clawState = false;
    }

    public boolean getClawState() {
        return clawState;
    }
}
