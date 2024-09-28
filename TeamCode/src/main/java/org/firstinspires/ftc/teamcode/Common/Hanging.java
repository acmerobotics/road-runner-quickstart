package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanging {
    SimpleServo hanging;

    public Hanging(SimpleServo h){
        hanging = h;
    }

    public Hanging(HardwareMap hardwareMap){
        hanging = new SimpleServo(hardwareMap, Constants.HangingConfigName, 0.0, 1.0);
    }

    public void hangingDown() {
        hanging.setPosition(Constants.HangingDown);
    }

    public void hangingUp() {
        hanging.setPosition(Constants.HangingUp);
    }

    public void hangingMiddle() {
        hanging.setPosition(Constants.HangingMiddle);
    }
}
