package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeController {
    DcMotorEx motor;
    public static double speed1 = 0.8, speed2 = -0.5;
    public IntakeController(HardwareMap map) {
        motor = map.get(DcMotorEx.class, "");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnOn() {
        motor.setPower(speed1);
    }

    public void turnOff() {
        motor.setPower(0);
    }

    public void reverse() {
        motor.setPower(speed2);
    }
}
