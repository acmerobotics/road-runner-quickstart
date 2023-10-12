package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RunToPositionMotorUtil {

    public void motorToPosition(DcMotorEx motor, double speed, int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(Math.abs(speed));

        if (motor.isBusy()) {

        } else {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
