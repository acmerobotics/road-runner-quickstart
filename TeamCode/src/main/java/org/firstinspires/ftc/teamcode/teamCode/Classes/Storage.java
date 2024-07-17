package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Storage {
    Servo servo;
    public static double openPos = 0.5;
    public static double closedPos = 0.5;
    public Storage(HardwareMap map) {
        servo = map.get(Servo.class, "");
    }

    public enum State {
        EMPTY,
        PIXEL,
        FULL
    }
    public State currentState = State.EMPTY;

    public void open() {
        servo.setPosition(openPos);
    }

    public void close() {
        servo.setPosition(closedPos);
    }
}
