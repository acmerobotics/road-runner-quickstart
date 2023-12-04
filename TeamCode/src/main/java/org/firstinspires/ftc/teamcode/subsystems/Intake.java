package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Intake extends Mechanism {

    CRServo intake; // Intake declaration

    @Override
    public void init(HardwareMap hwMap) {
        intake = hwMap.get(CRServo.class, ConfigInfo.intake.getDeviceName());
    }

    @Override
    public void loop(Gamepad gamepad) {

    }

    /**
     * Turns on the intake at the certain speed
     *
     * @param speed The speed that the intake will intake at. Will always end up
     *              positive
     */
    public void intake(double speed) {
        intake.setPower(Math.abs(speed));
    }

    /**
     * Turns on the outtake at the certain speed
     *
     * @param speed The speed that the intake will outtake at. Will always end up
     *              negative
     */
    public void outtake(double speed) {
        intake.setPower(-Math.abs(speed));
    }

    /**
     * Stops the intake
     */
    public void stop() {
        intake.setPower(0);
    }
}
