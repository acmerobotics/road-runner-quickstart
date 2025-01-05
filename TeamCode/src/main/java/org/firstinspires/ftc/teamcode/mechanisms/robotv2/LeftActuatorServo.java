package org.firstinspires.ftc.teamcode.mechanisms.robotv2;




import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LeftActuatorServo {
    public Servo leftActuator       = null;
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double LEFT_ACTUATOR_HORIZONTAL   = 0;
    public static double LEFT_ACTUATOR_VERTICAL  = .95;

    public static double LEFT_ACTUATOR_SCALE_MIN = 0.35;

    public static double LEFT_ACTUATOR_SCALE_MAX = 0.65;

    public LeftActuatorServo(HardwareMap hardwareMap) {
        leftActuator = hardwareMap.get(Servo.class, "leftActuatorServo");
        leftActuator.scaleRange(LEFT_ACTUATOR_SCALE_MIN, LEFT_ACTUATOR_SCALE_MAX);
        leftActuator.setPosition(LEFT_ACTUATOR_HORIZONTAL);
    }
    public void leftactuatorservohorizontal() {
        leftActuator.setPosition(LEFT_ACTUATOR_HORIZONTAL);
    }

    public void leftactuatorservovertical() {
        leftActuator.setPosition(LEFT_ACTUATOR_VERTICAL);
    }


    public Action leftactuatorservoFoldInAction() {
        return new InstantAction(() -> leftActuator.setPosition(LEFT_ACTUATOR_HORIZONTAL));
    }

    public Action leftactuatorservoFoldOutAction() {
        return new InstantAction(() -> leftActuator.setPosition(LEFT_ACTUATOR_VERTICAL));
    }
    public Action leftactuatorservoLowScaleAction(){
        return new InstantAction(() -> leftActuator.setPosition(LEFT_ACTUATOR_SCALE_MIN));
    }
    public Action leftactuatorservoHighScaleAction(){
        return new InstantAction(() -> leftActuator.setPosition(LEFT_ACTUATOR_SCALE_MAX));
    }
}


