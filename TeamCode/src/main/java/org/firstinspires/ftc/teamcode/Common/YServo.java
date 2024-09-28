package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class YServo {
    SimpleServo YServo;

    public YServo(SimpleServo y){
        YServo = y;
    }

    public YServo(HardwareMap hardwareMap){
        YServo = new SimpleServo(hardwareMap, Constants.YServoConfigName, 0.0, 1.0);
    }
    public void YServoUp(){
        YServo.setPosition(Constants.YServoUp);
    }
    public void YServoUpAuto(){
        YServo.setPosition(Constants.YServoUpAuto);
    }
    public void YServoSetPos(double pos){
        YServo.setPosition(pos);
    }

    public void YServoDown(){
        YServo.setPosition(Constants.YServoDown);
    }
    public void YServoDownAuto(){
        YServo.setPosition(Constants.YServoDownAuto);
    }
}
