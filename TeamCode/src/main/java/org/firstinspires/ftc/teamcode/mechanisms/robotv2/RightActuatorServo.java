package org.firstinspires.ftc.teamcode.mechanisms.robotv2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RightActuatorServo {
    public Servo rightActuator       = null;
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double RIGHT_ACTUATOR_HORIZONTAL   = 0;
    public static double RIGHT_ACTUATOR_VERTICAL  = .95;

    public static double RIGHT_ACTUATOR_SCALE_MIN = 0.35;

    public static double RIGHT_ACTUATOR_SCALE_MAX = 0.65;

    public RightActuatorServo(HardwareMap hardwareMap) {
        rightActuator = hardwareMap.get(Servo.class, "rightActuatorServo");
        rightActuator.scaleRange(RIGHT_ACTUATOR_SCALE_MIN, RIGHT_ACTUATOR_SCALE_MAX);
        rightActuator.setPosition(RIGHT_ACTUATOR_HORIZONTAL);
    }

    public void rightactuatorhorizontal() {
        rightActuator.setPosition(RIGHT_ACTUATOR_HORIZONTAL);
    }

    public void rightactuatorvertical() {
        rightActuator.setPosition(RIGHT_ACTUATOR_VERTICAL);
    }


    public Action rightactuatorFoldInAction() {
        return new InstantAction(() -> rightActuator.setPosition(RIGHT_ACTUATOR_HORIZONTAL));
    }

    public Action rightactuatorFoldOutAction() {
        return new InstantAction(() -> rightActuator.setPosition(RIGHT_ACTUATOR_VERTICAL));
    }
    public Action rightactuatorLowScaleAction(){
        return new InstantAction(() -> rightActuator.setPosition(RIGHT_ACTUATOR_SCALE_MIN));
    }
    public Action rightactuatorHighScaleAction(){
        return new InstantAction(() -> rightActuator.setPosition(RIGHT_ACTUATOR_SCALE_MAX));
    }
}

