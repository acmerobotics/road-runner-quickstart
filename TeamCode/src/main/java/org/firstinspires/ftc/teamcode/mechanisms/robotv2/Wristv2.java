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
    public static double WRIST_UP = 1;
    public static double WRIST_DOWN = 0;

    public static double WRIST_VERTICAL = 0.71;

    public static double WRIST_PICKUP_GROUND_SAMPLE = 0.0;
    public static double WRIST_SCALE_MIN = 0.0;

    public static double WRIST_SCALE_MAX = 0.9;

    public Wristv2(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.scaleRange(WRIST_SCALE_MIN, WRIST_SCALE_MAX);
        wrist.setPosition(WRIST_DOWN);
    }

    public void WristUp() {
        wrist.setPosition(WRIST_UP);
    }

    public void WristDown() {
        wrist.setPosition(WRIST_DOWN);
    }
    public void WristVertical() {
        wrist.setPosition(WRIST_VERTICAL);
    }

    public Action wristFoldInAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_UP));
    }

    public Action wristFoldOutAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_DOWN));
    }
    public Action wristPickUpGroundSampleAction() {
        return new InstantAction(() -> wrist.setPosition(WRIST_PICKUP_GROUND_SAMPLE));
    }

}

