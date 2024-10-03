package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public Servo intakeServoLeft;
    public Servo intakeServoRight;
    public DcMotor intakeMotor;

    public Intake(HardwareMap HWMap){
        intakeServoLeft = HWMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = HWMap.get(Servo.class, "intakeServoRight");
        intakeMotor = HWMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runMotor() {
        if (intakeMotor.getPower() != 1.0) {
            intakeMotor.setPower(1.0);
        }
    }

    public void runMotorBack() {
        if (intakeMotor.getPower() != -1.0) {
            intakeMotor.setPower(-1.0);
        }
    }

    public void extend() {
        intakeServoLeft.setPosition(100);
        intakeServoRight.setPosition(100);
        intakeMotor.setPower(1.0);
    }

    public void retract() {
        intakeServoLeft.setPosition(0);
        intakeServoRight.setPosition(0);
    }

}
