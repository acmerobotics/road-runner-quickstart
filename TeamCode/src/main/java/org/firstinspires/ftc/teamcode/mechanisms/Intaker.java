package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intaker {
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    public DcMotor intakeMotor;

    public static double flipTarget = 0.913;
    public static double flopTarget = 0.26;
    public static double middleTarget = 0.75;
    public static double perfectMid = 0.5;
    public static double intakePower = 0.9;
    public static double extakePower = 0.5;

    public Intaker(HardwareMap HWMap){
        intakeServoLeft = HWMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = HWMap.get(Servo.class, "intakeServoRight");
        intakeMotor = HWMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public class Flip implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServoLeft.setPosition(flipTarget);
            intakeServoRight.setPosition(1 - flipTarget);

            return false;
        }
    }
    public Action flip() {
        return new Flip();
    }

    public class Middle implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServoLeft.setPosition(middleTarget);
            intakeServoRight.setPosition(1 - middleTarget);

            return false;
        }
    }
    public Action middle() {
        return new Middle();
    }


    public class Flop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServoLeft.setPosition(flopTarget);
            intakeServoRight.setPosition(1 - flopTarget);
            return false;
        }
    }
    public Action flop() {
        return new Flop();
    }

    public class Flat implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServoLeft.setPosition(perfectMid);
            intakeServoRight.setPosition(1 - perfectMid);
            return false;
        }
    }
    public Action flat() {
        return new Flat();
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
            intakeMotor.setPower(-intakePower);
            return false;
        }
    }
    public Action intake() {
        return new Intake();
    }

    public class Extake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(extakePower);
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
            intakeMotor.setPower(-0.2);
            return false;
        }
    }
    public Action creep() {
        return new Creep();
    }

    public void downExtake() {
        intakeMotor.setPower(extakePower);
    }

    public void downIntake() {
        intakeMotor.setPower(-intakePower);
    }
}
