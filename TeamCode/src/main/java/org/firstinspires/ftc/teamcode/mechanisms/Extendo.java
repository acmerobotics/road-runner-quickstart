package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extendo {
    public DcMotor extendoLeft;
    public DcMotor extendoRight;

    public PIDCoefficients extendoLeftCoeffs = new PIDCoefficients(1, 0 , 0);
    public PIDCoefficients extendoRightCoeffs = new PIDCoefficients(1, 0 , 0);
    public PIDFController extendoLeftPID = new PIDFController(extendoLeftCoeffs);
    public PIDFController extendoRightPID = new PIDFController(extendoRightCoeffs);

    public boolean moving = false;
    public Extendo(HardwareMap HWMap){
        extendoLeft = HWMap.get(DcMotor.class, "extendoLeft");
        extendoRight = HWMap.get(DcMotor.class, "extendoRight");

        extendoLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendoRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }

    public void urMom(double pos) {
        extendoLeftPID.setTargetPosition(pos);
        extendoRightPID.setTargetPosition(pos);
        moving = true;
    }

    public double getPos() {
        return (double) (extendoLeft.getCurrentPosition() + extendoRight.getCurrentPosition()) / 2;
    }

    public void doStuff() {
        extendoLeft.setPower(extendoLeftPID.update(extendoLeft.getCurrentPosition()));
        extendoRight.setPower(extendoRightPID.update(extendoRight.getCurrentPosition()));

        if (Math.abs(extendoLeftPID.getTargetPosition() - getPos()) < 10) {
            moving = false;
        }

    }
}


