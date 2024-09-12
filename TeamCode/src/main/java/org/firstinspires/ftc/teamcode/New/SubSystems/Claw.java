package org.firstinspires.ftc.teamcode.New.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Claw {
    enum clawPositions {
        Closed(.9), Open(.5);

        clawPositions(double position) {
            position = this.position;
        }
        double position;

    }

    HardwareMap hardwareMap;
    Servo rightClaw;
    public Servo leftClaw;

    public ElapsedTime timer;


    public Claw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        timer = new ElapsedTime();

    }

    public void update() {
        RunSequence();
    }

    private void closeClaw() {
        rightClaw.setPosition(clawPositions.Closed.position);
        rightClaw.setPosition(clawPositions.Closed.position);
    }

    private void openClaw() {
        rightClaw.setPosition(clawPositions.Open.position);
        rightClaw.setPosition(clawPositions.Open.position);
    }

    private void RunSequence() {

        double time = timer.seconds();
        if (time >= 4.0 && time <= 6.0) openClaw();

        if (time >= 8.0 && time <= 11.0) closeClaw();

        if (time >= 12.0) openClaw();

        if (time >= 15.0 && time <= 20.0) closeClaw();

    }


}
