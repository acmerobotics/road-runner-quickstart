package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extension {
    SimpleServo extension;

    public Extension(SimpleServo e){
        extension = e;
    }

    public Extension(HardwareMap hardwareMap){
        extension = new SimpleServo(hardwareMap, Constants.ExtensionName, 0.0, 1.0);
    }

    public double getExtensionPosition() {
        return extension.getPosition();
    }

    public void extend(){
        extension.setPosition(Constants.ExtensionOut);
    }

    public void retract(){
        extension.setPosition(Constants.ExtensionIn);
    }
}
