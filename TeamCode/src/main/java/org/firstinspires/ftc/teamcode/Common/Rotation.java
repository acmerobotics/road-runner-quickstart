package org.firstinspires.ftc.teamcode.Common;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Rotation {
    SimpleServo rotation;

    public Rotation(HardwareMap hardwareMap){

        rotation = new SimpleServo(hardwareMap, "CR", 0.0, 1.0);
    }

    public double getFourBarPosition() {
        return rotation.getPosition();
    }

    public void RotationHorizontalTele(){
        rotation.setPosition(1);
    }

    public void RotationVerticalTele(){
        rotation.setPosition(0.5);
    }

    public Action RotationHorizontal() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(1);
                return false;
            }
        };
    }

    public Action RotationVertical() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rotation.setPosition(0.5);
                return false;
            }
        };
    }
}
