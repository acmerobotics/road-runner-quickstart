package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class ContinuousServo extends Component{

    private float speed;
    public CRServo servo;

    public ContinuousServo(int port, String name, HardwareMap map){
        super(port, name);
        servo = map.crservo.get(name);
    }

    public void setSpeed(float speed) {
        this.speed = speed;
        servo.setPower(speed);
    }

    public float getSpeed(){
        return (float) servo.getPower();
    }

}