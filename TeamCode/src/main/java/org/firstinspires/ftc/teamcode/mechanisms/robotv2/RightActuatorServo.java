package org.firstinspires.ftc.teamcode.mechanisms.robotv2;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RightActuatorServo {
    public Servo servo = null;
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double RIGHT_ACTUATOR_HORIZONTAL   = 0.5;
    public static double RIGHT_ACTUATOR_VERTICAL  = .75;
    public static double RIGHT_ACTUATOR_HANG = 0.55;

    public static double RIGHT_ACTUATOR_SCALE_MIN = 0.0;

    public static double RIGHT_ACTUATOR_SCALE_MAX = 1.0;

    public RightActuatorServo(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "rightActuatorServo");
        servo.scaleRange(RIGHT_ACTUATOR_SCALE_MIN, RIGHT_ACTUATOR_SCALE_MAX);
        servo.setPosition(RIGHT_ACTUATOR_HORIZONTAL);
    }

    public void setHorizontal() {
        servo.setPosition(RIGHT_ACTUATOR_HORIZONTAL);
    }

    public void setVertical() {
        servo.setPosition(RIGHT_ACTUATOR_VERTICAL);
    }
    public void setHanging(){
        servo.setPosition(RIGHT_ACTUATOR_HANG);
    }
}

