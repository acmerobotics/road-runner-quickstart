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
    MotorGroup Extension;

    MotorEx ExtensionRight;
    MotorEx ExtensionLeft;

    public Extension(MotorGroup e){
        Extension = e;
    }

    public Extension(HardwareMap hardwareMap){
        MotorEx EL = new MotorEx(hardwareMap, Constants.ExtensionLeftName);
        EL.setRunMode(Motor.RunMode.RawPower);
        EL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.ExtensionLeft = EL;

        MotorEx ER = new MotorEx(hardwareMap, Constants.ExtensionRightName);
        ER.setRunMode(Motor.RunMode.RawPower);
        ER.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.ExtensionRight = ER;

        Extension = new MotorGroup(this.ExtensionLeft, this.ExtensionRight);



//        extension = new MotorEx(hardwareMap, "EL");
    }

    public Action Extend (){
        return new Action (){
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                long startTime = 0;
                if (!initialized) {
                    ExtensionRight.set(1);
                    ExtensionLeft.set(-1);
                    initialized = true;
                    startTime = System.currentTimeMillis();
                }
                double pos = ExtensionLeft.getCurrentPosition();
                //TODO: replace time with actual time
//                final int extendOutTime = 1500;

//                try {
//                    Thread.sleep(Constants.ExtensionTime);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
                if (System.currentTimeMillis() >= startTime + Constants.ExtensionTime) {
                    packet.put("extendPos", System.currentTimeMillis() - startTime );
                    return true;

                }
                else {
                    ExtensionLeft.set(0);
                    ExtensionRight.set(0);
                    return false;
                }

            }

        };


    }

    public Action Retract (){
        return new Action (){
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                long startTime = 0;
                if (!initialized) {
                    ExtensionRight.set(-1);
                    ExtensionLeft.set(1);
                    initialized = true;
                    startTime = System.currentTimeMillis();
                }
                double pos = ExtensionLeft.getCurrentPosition();
                //TODO: replace time with actual time
//                final int extendOutTime = 1500;

//                try {
//                    Thread.sleep(Constants.ExtensionTime);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
                if (System.currentTimeMillis() >= startTime + Constants.ExtensionTime) {
                    packet.put("extendPos", System.currentTimeMillis() - startTime );
                    return true;

                }
                else {
                    ExtensionLeft.set(0);
                    ExtensionRight.set(0);
                    return false;
                }

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
