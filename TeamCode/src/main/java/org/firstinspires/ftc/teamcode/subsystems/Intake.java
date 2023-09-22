package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Intake extends Mechanism {

    CRServo intake;

    public String intakeName = "intake";

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(CRServo.class, intakeName);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.a) {
            intake.setPower(1);
        } else if (gamepad.b) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }
}
