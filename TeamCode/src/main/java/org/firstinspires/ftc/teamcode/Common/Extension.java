package org.firstinspires.ftc.teamcode.Common;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;


public class Extension {
    SimpleServo ExtensionRight;
    SimpleServo ExtensionLeft;

    public Extension(HardwareMap hardwareMap){
        ExtensionLeft = new SimpleServo(hardwareMap, "ExtL", 0, 0.5);
        ExtensionRight = new SimpleServo(hardwareMap, "ExtR", 0, 0.5);
    }

    public Action Extend (){
        return new Action (){
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ExtensionRight.setPosition(0.7);
                ExtensionLeft.setPosition(0.3);
                return true;
            }

        };


    }

    public Action Retract (){
        return new Action (){
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ExtensionRight.setPosition(1);
                ExtensionLeft.setPosition(0);
                return true;
            }

        };


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
