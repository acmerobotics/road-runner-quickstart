package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;





public class Slides extends Mechanism {

    DcMotor Slides;

    public String slidesName = "Slides";

    @Override
    public void init(HardwareMap hwMap) {
        Slides = hwMap.get(DcMotor.class, slidesName);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.a) {
            Slides.setPower(1);
        } else if (gamepad.b) {
            Slides.setPower(-1);
        } else {
            Slides.setPower(0);
        }
    }
}