package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    DcMotor slidesLeftMotor;
    DcMotor slidesRightMotor;

    public PIDCoefficients slidesLeftCoeffs = new PIDCoefficients(1, 0 , 0);
    public PIDCoefficients slidesRightCoeffs = new PIDCoefficients(1, 0 , 0);
    public PIDFController slidesLeftPID = new PIDFController(slidesLeftCoeffs);
    public PIDFController slidesRightPID = new PIDFController(slidesRightCoeffs);

    public boolean moving = false;

    public Slides(HardwareMap HWMap){
        slidesLeftMotor = HWMap.get(DcMotor.class, "leftSlidesMotor");
        slidesRightMotor = HWMap.get(DcMotor.class, "rightSlidesMotor");
    }

    public void slide(double pos) {
        slidesLeftPID.setTargetPosition(pos);
        slidesRightPID.setTargetPosition(pos);
        moving = true;
    }

    public double getPos() {
        return (double) (slidesLeftMotor.getCurrentPosition() + slidesRightMotor.getCurrentPosition()) / 2;
    }

    public void updatePID() {
        slidesLeftPID.update(slidesLeftMotor.getCurrentPosition());
        slidesRightPID.update(slidesRightMotor.getCurrentPosition());
        if (Math.abs(slidesLeftPID.getTargetPosition() - getPos()) < 10) {
            moving = false;
        }
    }
}
