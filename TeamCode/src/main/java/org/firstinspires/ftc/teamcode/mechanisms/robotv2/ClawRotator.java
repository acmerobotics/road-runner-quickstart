package org.firstinspires.ftc.teamcode.mechanisms.robotv2;





import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawRotator {
    public Servo servo = null; //the wrist servo

    public static double CLAW_RESET_POS = 0.555;
    public static double CLAW_ROTATE_45 = .032;

    public ClawRotator(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "clawrotator");
        servo.setPosition(CLAW_RESET_POS);
    }

    public void rotateLeft45(){
        servo.setPosition(CLAW_RESET_POS + CLAW_ROTATE_45);
    }

    public void rotateRight45(){
        servo.setPosition(CLAW_RESET_POS - CLAW_ROTATE_45);
    }

    public void rotateZero(){
        servo.setPosition(CLAW_RESET_POS);
    }
    public void rotate90(){
        servo.setPosition(CLAW_RESET_POS + 2*CLAW_ROTATE_45);
    }

    // Auto Actions
    public Action clawRotateResetAction() {
        return new InstantAction(() -> servo.setPosition(CLAW_RESET_POS));
    }

    public Action clawRotate45LeftAction() {
        return new InstantAction(() -> servo.setPosition(CLAW_RESET_POS + CLAW_ROTATE_45));
    }
    public Action clawRotate45RightAction() {
        return new InstantAction(() -> servo.setPosition(CLAW_RESET_POS - CLAW_ROTATE_45));
    }

}


