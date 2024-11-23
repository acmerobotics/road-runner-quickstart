package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wrist {
    public Servo wrist       = null; //the wrist servo
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double WRIST_FOLDED_IN   = 1;
    public static double WRIST_FOLDED_OUT  = 0;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(WRIST_FOLDED_IN);
    }

    public void foldIn() {
        wrist.setPosition(WRIST_FOLDED_IN);
    }

    public void foldOut() {
        wrist.setPosition(WRIST_FOLDED_OUT);
    }

}
