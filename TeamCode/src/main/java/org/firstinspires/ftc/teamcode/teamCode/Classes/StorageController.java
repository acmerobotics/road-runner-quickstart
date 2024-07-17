package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StorageController {
    Servo servo;

    public StorageController(HardwareMap map) {
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
}
