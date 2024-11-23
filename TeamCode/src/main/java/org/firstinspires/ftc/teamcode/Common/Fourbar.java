package org.firstinspires.ftc.teamcode.Common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Fourbar {
    SimpleServo four_bar;

    public Fourbar(HardwareMap hardwareMap){

        four_bar = new SimpleServo(hardwareMap, "VFBL", 0.0, 1.0);
    }

    public double getFourBarPosition() {
        return four_bar.getPosition();
    }

    public void FourBarUpTele(){
        four_bar.setPosition(0.9);
    }

    public void FourBarDownTele(){
        four_bar.setPosition(0.5);
    }

    public Action FourBarUp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                four_bar.setPosition(0.9);
                return false;
            }
        };
    }

    public Action FourBarDown() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                four_bar.setPosition(0.5);
                return false;
            }
        };
    }
}
