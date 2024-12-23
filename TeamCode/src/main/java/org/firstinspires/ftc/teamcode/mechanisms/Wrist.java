package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wrist {
    public Servo wrist       = null; //the wrist servo
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double WRIST_FOLDED_IN   = 0.2;
    public static double WRIST_FOLDED_OUT  = 0.8;

    public static double WRIST_SCALE_MIN = 0.42;

    public static double WRIST_SCALE_MAX = 0.7;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.scaleRange(WRIST_SCALE_MIN, WRIST_SCALE_MAX);
        wrist.setPosition(WRIST_FOLDED_IN);
    }

    public void foldIn() {
        wrist.setPosition(WRIST_FOLDED_IN);
    }

    public void foldOut() {
        wrist.setPosition(WRIST_FOLDED_OUT);
    }

    public Action wristFoldInAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_FOLDED_IN));
    }

    public Action wristFoldOutAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_FOLDED_OUT));
    }
    public Action wristLowScaleAction(){
        return new InstantAction(() -> wrist.setPosition(WRIST_SCALE_MIN));
    }
    public Action wristHighScaleAction(){
        return new InstantAction(() -> wrist.setPosition(WRIST_SCALE_MAX));
    }
}

