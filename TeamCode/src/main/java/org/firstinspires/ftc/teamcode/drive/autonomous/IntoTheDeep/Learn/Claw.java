package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;

//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class Claw {
//    private final Servo leftServo, rightServo;
//    double closedPos = 0.65;
//    double openPos = 0.4;
//
//    public Claw(HardwareMap hardwareMap) {
//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        rightServo = hardwareMap.get(Servo.class, "rightServo");
//    }
//
//    public void open() {
//        leftServo.setPosition(openPos);
//        rightServo.setPosition(openPos);
//    }
//
//    public void close() {
//        leftServo.setPosition(closedPos);
//        rightServo.setPosition(closedPos);
//    }
//}

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo clawLeft, clawRight;

    public Claw(HardwareMap hardwareMap) {
        clawLeft = hardwareMap.get(Servo.class, "leftClaw");
        clawRight = hardwareMap.get(Servo.class, "rightClaw");
        clawLeft.setDirection(Servo.Direction.REVERSE);
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double closedPos = 0.65;
            clawLeft.setPosition(closedPos);
            clawRight.setPosition(closedPos);
            return false;
        }
    }

    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double openPos = 0.4;
            clawLeft.setPosition(openPos);
            clawRight.setPosition(openPos);
            return false;
        }
    }

    public Action openClaw() {
        return new OpenClaw();
    }
}

