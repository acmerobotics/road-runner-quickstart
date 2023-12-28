package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Mechanism;


public class Hanger extends Mechanism{

    private DcMotorEx hanger;
    public String hangerName = "Hanger";
    @Override
    public void init(HardwareMap hwMap) {
        hanger = hwMap.get(DcMotorEx.class, hangerName);
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanger.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            hanger.setPower(1);
        } else if (gamepad.left_bumper) {
            hanger.setPower(-1);
        } else {
            hanger.setPower(0);
        }
    }
}
