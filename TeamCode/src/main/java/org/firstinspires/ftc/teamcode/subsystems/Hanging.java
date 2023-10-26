package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Mechanism;


public class Hanging extends Mechanism{

    DcMotor Hanger;
    public String HangerName = "Hanger";
    @Override
    public void init(HardwareMap hwMap) {
        Hanger = hwMap.get(DcMotor.class, HangerName);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            Hanger.setPower(1);
        } else if (gamepad.left_bumper) {
            Hanger.setPower(-1);
        } else {
            Hanger.setPower(0);
        }
    }
}
