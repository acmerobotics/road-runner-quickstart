package org.firstinspires.ftc.teamcode.mechanisms.robotv2;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wristv2 {
    public Servo wrist = null; //the wrist servo
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double WRIST_FOLD_OUT = 0.6;
    public static double WRIST_FOLD_IN = 0.36;
    public static double WRIST_VERTICAL = 0.5;

    public static double WRIST_SAMPLE_3 = 0.4;

    public static double WRIST_PICKUP_GROUND_SAMPLE = WRIST_FOLD_IN;
    public static double WRIST_SCALE_MIN = 0.0;

    public static double WRIST_SCALE_MAX = 1.0;

    public Wristv2(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.scaleRange(WRIST_SCALE_MIN, WRIST_SCALE_MAX);
        wrist.setPosition(WRIST_FOLD_IN);
    }

    public void WristFoldOut() {
        wrist.setPosition(WRIST_FOLD_OUT);
    }

    public void WristFoldIn() {
        wrist.setPosition(WRIST_FOLD_IN);
    }
    public void WristVertical() {
        wrist.setPosition(WRIST_VERTICAL);
    }

    public Action wristFoldInAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_FOLD_IN));
    }

    public Action wristFoldOutAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_FOLD_OUT));
    }
    public Action wristVerticalAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_VERTICAL));
    }
    public Action wristPickUpGroundSampleAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_PICKUP_GROUND_SAMPLE));
    }

    public Action wristPickUpSample3Action(){
        return new InstantAction(() -> wrist.setPosition(WRIST_SAMPLE_3));
    }

}

