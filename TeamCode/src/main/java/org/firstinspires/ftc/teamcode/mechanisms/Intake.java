package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public Servo intakeServoLeft;
    public Servo intakeServoRight;
    public DcMotor intakeMotor;
    public static boolean flipped = false;

    public Intake(HardwareMap HWMap){
        intakeServoLeft = HWMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = HWMap.get(Servo.class, "intakeServoRight");
        intakeMotor = HWMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class Flip implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //TODO: set values to actual servo positions
            intakeServoLeft.setPosition(1);
            intakeServoRight.setPosition(1);
            //intakeMotor.setPower(1.0);
            flipped = true;
            return false;
        }
    }
    public Action flip() {
        return new Flip();
    }

    public class Flop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //TODO: set values to actual servo positions
            intakeServoLeft.setPosition(0);
            intakeServoRight.setPosition(0);
            flipped = false;
            return false;
        }
    }
    public Action flop() {
        return new Flop();
    }
}
