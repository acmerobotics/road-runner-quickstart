package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    //private Servo bucketServo;;
    private Servo clawServo;
    private Servo wallServo;

    public static double flipTarget = 0.3;
    public static double flopTarget = 0.1;
    public static double upTarget = 0.07;
    public static double closeTarget = 0.6;
    public static double openTarget = 0.2;
    public static double wallopenTarget = 0.9;
    public static double wallcloseTarget = 0.41;

    public Claw(HardwareMap HWMap) {
        //bucketServo = HWMap.get(Servo.class, "bucketServo");
        clawServo = HWMap.get(Servo.class, "clawServo");
        wallServo = HWMap.get(Servo.class, "wallServo");
    }


    public class Flip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //bucketServo.setPosition(flipTarget);
            return false;
        }
    }

    public Action flip() {
        return new Flip();
    }

    public class Flop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //bucketServo.setPosition(flopTarget);
            return false;
        }
    }

    public Action flop() {
        return new Flop();
    }

    public class Close implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(closeTarget);
            wallServo.setPosition(wallcloseTarget);
            return false;
        }
    }

    public Action close() {
        return new Close();
    }

    public class Open implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(openTarget);
            wallServo.setPosition(wallopenTarget);
            return false;
        }
    }

    public Action open() {
        return new Open();
    }

    public class Up implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //bucketServo.setPosition(upTarget);
            return false;
        }
    }

    public Action up() {
        return new Up();
    }
}

