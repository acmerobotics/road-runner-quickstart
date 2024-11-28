package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intaker {
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    public DcMotor intakeMotor;

    public Intaker(HardwareMap HWMap){
//        intakeServoLeft = HWMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = HWMap.get(Servo.class, "intakeServoRight");
        intakeMotor = HWMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public class Flip implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            intakeServoLeft.setPosition(0.95);
            intakeServoRight.setPosition(0.05);

            return false;
        }
    }
    public Action flip() {
        return new Flip();
    }

    public class Middle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            intakeServoLeft.setPosition(0.65);
            intakeServoRight.setPosition(0.35);

            return false;
        }
    }
    public Action middle() {
        return new Middle();
    }


    public class Flop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            intakeServoLeft.setPosition(0.17);
            intakeServoRight.setPosition(0.85);
            return false;
        }
    }
    public Action flop() {
        return new Flop();
    }

    public class UpFlip implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServoLeft.setPosition(0.9);
            intakeServoRight.setPosition(0.1);
            return false;
        }
    }
    public Action upFlip() {
        return new UpFlip();
    }

    public class Intake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(-0.9);
            return false;
        }
    }
    public Action intake() {
        return new Intake();
    }

    public class Extake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(0.325);
            return false;
        }
    }
    public Action extake() {
        return new Extake();
    }

    public class Off implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(0);
            return false;
        }
    }
    public Action off() {
        return new Off();
    }

    public class Creep implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(-0.5);
            return false;
        }
    }
    public Action creep() {
        return new Creep();
    }

    public void downExtake() {
        intakeMotor.setPower(0.45);
    }

    public void downIntake() {
        intakeMotor.setPower(-0.9);
    }
}
