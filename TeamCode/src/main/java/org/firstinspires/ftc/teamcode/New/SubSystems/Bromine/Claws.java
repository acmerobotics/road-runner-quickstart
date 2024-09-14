package org.firstinspires.ftc.teamcode.New.SubSystems.Bromine;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.New.SubSystems.Java.JavaSubsystems;


public class Claws implements JavaSubsystems {
    @Override
    public void update() {
        leftClaw.update();
        rightClaw.update();
    }

    public LeftClaw leftClaw;
    public RightClaw rightClaw;


    public Claws(HardwareMap hardwareMap) {
        leftClaw = new LeftClaw(hardwareMap);
        rightClaw = new RightClaw(hardwareMap);
    }

    public static class LeftClaw implements JavaSubsystems {

        @Override
        public void update() {
            switch (state) {
                case Closed:
                    leftClaw.setPosition(States.Closed.pos);
                    break;
                case Open:
                    leftClaw.setPosition(States.Open.pos);
                    break;
            }
        }

        public States state = States.Open;
        Servo leftClaw;

        public enum States {
            Closed(.9), Open(.5);

            States(double pos) {
                this.pos = pos;
            }

            final double pos;
        }

        public LeftClaw(HardwareMap hardwareMap) {
            leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        }

    }
    public static class RightClaw implements JavaSubsystems {

        @Override
        public void update() {
            switch (state) {
                case Closed:
                    rightClaw.setPosition(States.Closed.pos);
                    break;
                case Open:
                    rightClaw.setPosition(States.Open.pos);
                    break;
            }
        }

        public States state = States.Open;
        Servo rightClaw;

        public enum States {
            Closed(.9), Open(.5);

            States(double pos) {
                this.pos = pos;
            }

            final double pos;
        }

        public RightClaw(HardwareMap hardwareMap) {
            rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        }
    }
}
