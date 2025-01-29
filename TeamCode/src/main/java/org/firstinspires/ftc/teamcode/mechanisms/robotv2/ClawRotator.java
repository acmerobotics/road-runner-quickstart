package org.firstinspires.ftc.teamcode.mechanisms.robotv2;





import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawRotator {
    public Servo claw = null; //the wrist servo
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double CLAW_ROTATE_0 = 0.5;
    public static double CLAW_ROTATE_45_LEFT = CLAW_ROTATE_0 + 45/270.;
    public static double CLAW_ROTATE_45_RIGHT = CLAW_ROTATE_0 - 45/270.;

    public static double CLAW_ROTATE_90 = 1;

    public ClawRotator(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "clawrotator");
        claw.setPosition(CLAW_ROTATE_0);
    }

    public void setClawRotate0() {
        claw.setPosition(CLAW_ROTATE_0);
    }

    public void setClawRotate45Left() {
        claw.setPosition(CLAW_ROTATE_45_LEFT);
    }
    public void setClawRotate45Right() {
        claw.setPosition(CLAW_ROTATE_45_RIGHT);
    }
    public void setClawRotate90() {
        claw.setPosition(CLAW_ROTATE_90);
    }

    public Action clawRotate0Action() {
        return new InstantAction(() -> claw.setPosition(CLAW_ROTATE_0));
    }

    public Action clawRotate45LeftAction() {
        return new InstantAction(() -> claw.setPosition(CLAW_ROTATE_45_LEFT));
    }
    public Action clawRotate45RightAction() {
        return new InstantAction(() -> claw.setPosition(CLAW_ROTATE_45_RIGHT));
    }
    public Action clawRotate90Action() {
        return new InstantAction(() -> claw.setPosition(CLAW_ROTATE_90));
    }
}


