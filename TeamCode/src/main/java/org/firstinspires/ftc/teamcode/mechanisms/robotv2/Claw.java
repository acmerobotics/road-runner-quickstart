package org.firstinspires.ftc.teamcode.mechanisms.robotv2;





import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    public Servo claw = null; //the wrist servo
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double CLAW_CLOSED = 1;
    public static double CLAW_OPEN = 0;


    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_CLOSED);
    }


    public void clawClose() {
        claw.setPosition(CLAW_CLOSED);
    }

    public void clawOpen() {
        claw.setPosition(CLAW_OPEN);
    }

    public Action clawFoldInAction() {
        return new InstantAction(() -> claw.setPosition(CLAW_CLOSED));
    }

    public Action clawFoldOutAction() {
        return new InstantAction(() -> claw.setPosition(CLAW_OPEN));
    }
}


