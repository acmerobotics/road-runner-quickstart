package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    DcMotor Spinner;
    IntakeState intakeState;
    Servo Flipper1;
    FlipperState flipperState;
    Servo Flipper2;


    public Intake (HardwareMap hardwareMap){
        Spinner = hardwareMap.get(DcMotor.class, "intake");
        Flipper1 = hardwareMap.get(Servo.class, "flipper1");
        Flipper2 = hardwareMap.get(Servo.class, "flipper2");
    }

    public void update(){
        Spinner.setPower(intakeState.power);
        Flipper1.setPosition(flipperState.pos);
        Flipper2.setPosition(-flipperState.pos);
    }
    enum IntakeState{
        IN (1),
        OUT (-1),
        STOPPED (0);

        final double power;
        IntakeState(double power) {
            this.power = power;
        }
    }

    enum FlipperState{
        COLLECTING (0.5),
        TRANSFERRING (0),
        STATIONARY (0.25);

        final double pos;
        FlipperState(double pos) {
            this.pos = pos;
        }
    }
}
