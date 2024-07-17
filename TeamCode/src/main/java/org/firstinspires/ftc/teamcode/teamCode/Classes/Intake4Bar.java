package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake4Bar {
    Servo servo;

    public static double poz1 = 0.5;
    public static double poz2 = 0.5;
    public static double poz3 = 0.5;
    public static double poz4 = 0.5;
    public static double poz5 = 0.5;
    public static double movingPos = 0.5;


    public Intake4Bar(HardwareMap map) {
        servo = map.get(Servo.class, "");
    }

    public enum POSE {
        pixel1,
        pixel2,
        pixel3,
        pixel4,
        pixel5,
        moving
    }

    public void goTo(POSE poz) {
        switch (poz) {
            case pixel1: {
                servo.setPosition(poz1);
                break;
            }
            case pixel2: {
                servo.setPosition(poz2);
                break;
            }
            case pixel3: {
                servo.setPosition(poz3);
                break;
            }
            case pixel4: {
                servo.setPosition(poz4);
                break;
            }
            case pixel5: {
                servo.setPosition(poz5);
                break;
            }
            case moving: {
                servo.setPosition(movingPos);
                break;
            }

        }
    }
}
