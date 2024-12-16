package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo bucketServo;;
    private Servo clawServo;

    public Claw(HardwareMap HWMap) {
        bucketServo = HWMap.get(Servo.class, "bucketServo");
        clawServo = HWMap.get(Servo.class, "clawServo");
    }


    public class Flip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            bucketServo.setPosition(0.3);
            return false;
        }
    }

    public Action flip() {
        return new Flip();
    }

    public class Flop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            bucketServo.setPosition(0);
            return false;
        }
    }

    public Action flop() {
        return new Flop();
    }

    public class Close implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(0.25);
            return false;
        }
    }

    public Action close() {
        return new Close();
    }

    public class Open implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(0);
            return false;
        }
    }

    public Action open() {
        return new Open();
    }

    public class Up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            bucketServo.setPosition(0.07);
//            bucketRightServo.setPosition(0.5);
            return false;
        }
    }

    public Action up() {
        return new Up();
    }
}

