package org.firstinspires.ftc.teamcode.mechanisms.robotv2;




import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class LeftActuatorServo {
    public Servo servo = null;
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double LEFT_ACTUATOR_HORIZONTAL   = 0.5;
    public static double LEFT_ACTUATOR_VERTICAL  = .8;
    public static double LEFT_ACTUATOR_HANG  = .5;

    public static double LEFT_ACTUATOR_SCALE_MIN = 0.0;

    public static double LEFT_ACTUATOR_SCALE_MAX = 1.0;

    public LeftActuatorServo(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "leftActuatorServo");
        servo.setDirection(Servo.Direction.REVERSE);
        servo.scaleRange(LEFT_ACTUATOR_SCALE_MIN, LEFT_ACTUATOR_SCALE_MAX);
        servo.setPosition(LEFT_ACTUATOR_HORIZONTAL);
    }
    public void setHorizontal() {
        servo.setPosition(LEFT_ACTUATOR_HORIZONTAL);
    }

    public void setVertical() {
        servo.setPosition(LEFT_ACTUATOR_VERTICAL);
    }

    public void setHanging(){
        servo.setPosition(LEFT_ACTUATOR_HANG);
    }
}


