package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extension {
    MotorEx extension;

    public Extension(MotorEx e){
        extension = e;
    }

    public Extension(HardwareMap hardwareMap){
        extension = new MotorEx(hardwareMap, "EL");
    }

//    public double getExtensionPosition() {
//        return extension.getPosition();
//    }
//
//    public void extend(){
//        extension.setPosition(Constants.ExtensionOut);
//    }
//
//    public void retract(){
//        extension.setPosition(Constants.ExtensionIn);
//    }
}
