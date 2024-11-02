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
    private DcMotor intakeMotor;

    public Intaker(HardwareMap HWMap){
        intakeServoLeft = HWMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = HWMap.get(Servo.class, "intakeServoRight");
        intakeMotor = HWMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public class Flip implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServoLeft.setPosition(0.87);
            intakeServoRight.setPosition(0.13);

            return false;
        }
    }
    public Action flip() {
        return new Flip();
    }

    public class Flop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServoLeft.setPosition(0.05);
            intakeServoRight.setPosition(0.95);
            return false;
        }
    }
    public Action flop() {
        return new Flop();
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
            intakeMotor.setPower(0.3);
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


}
