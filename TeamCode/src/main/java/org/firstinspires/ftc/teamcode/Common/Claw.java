package org.firstinspires.ftc.teamcode.Common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    SimpleServo Claw;

    public Claw(SimpleServo c, SimpleServo c2){
        Claw = c;
    }

    public Claw(HardwareMap hardwareMap){
        Claw = new SimpleServo(hardwareMap, "CL", 0.0, 1.0);
    }

    public double getClawPosition() {
        return Claw.getPosition();
    }
    public void ClawOpenTele(){
        Claw.setPosition(0.8);
    }

    public void ClawCloseTele(){
        Claw.setPosition(0);
    }

    public Action ClawClose() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Claw.setPosition(0);
                return false;
            }
        };
    }

    public Action ClawOpen() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Claw.setPosition(1);
                return false;
            }
        };
    }
}
