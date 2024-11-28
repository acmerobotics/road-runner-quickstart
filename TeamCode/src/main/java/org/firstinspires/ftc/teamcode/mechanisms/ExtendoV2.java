package org.firstinspires.ftc.teamcode.mechanisms;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PIDFController;

public class ExtendoV2 {
    public Servo extendoLeftServo;
    public Servo extendoRightServo;

    public double pos;

    public ExtendoV2(HardwareMap HWMap) {
        extendoLeftServo = HWMap.get(Servo.class, "extendoLeftServo");
        extendoRightServo = HWMap.get(Servo.class, "extendoRightServo");


    }

    public class Retract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pos = 0;
            extendoLeftServo.setPosition(pos);
            extendoRightServo.setPosition(pos);
            return false;
        }
    }

    public Action retract() {
        return new Retract();
    }

    public class Extend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pos = 0.2;
            extendoLeftServo.setPosition(pos);
            extendoRightServo.setPosition(pos);
            return false;
        }
    }

    public Action extend() {
        return new Extend();
    }

    public void move(double d) {
        pos += d;
        extendoLeftServo.setPosition(pos);
        extendoRightServo.setPosition(pos);
    }
}



