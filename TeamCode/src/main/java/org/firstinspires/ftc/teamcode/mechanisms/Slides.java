package org.firstinspires.ftc.teamcode.mechanisms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDFController;

public class Slides {
    DcMotor slidesLeftMotor;
    DcMotor slidesRightMotor;

    public PIDFController.PIDCoefficients slidesLeftCoeffs = new PIDFController.PIDCoefficients(1, 0 , 0);
    public PIDFController.PIDCoefficients slidesRightCoeffs = new PIDFController.PIDCoefficients(1, 0 , 0);
    public PIDFController slidesLeftPID = new PIDFController(slidesLeftCoeffs);
    public PIDFController slidesRightPID = new PIDFController(slidesRightCoeffs);

    public boolean moving = false;

    public Slides(HardwareMap HWMap){
        slidesLeftMotor = HWMap.get(DcMotor.class, "leftSlidesMotor");
        slidesRightMotor = HWMap.get(DcMotor.class, "rightSlidesMotor");
    }

    public class SlideTopBasket implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of top basket
                slidesLeftPID.setTargetPosition(200);
                slidesRightPID.setTargetPosition(200);
                init = true;
            }
            slidesLeftMotor.setPower(slidesLeftPID.update(slidesLeftMotor.getCurrentPosition()));
            slidesRightMotor.setPower(slidesRightPID.update(slidesRightMotor.getCurrentPosition()));

            if (Math.abs(slidesLeftPID.getTargetPosition() - getPos()) < 15) {
                return true;
            }
            return false;
        }
    }
    public Action slideTopBasket() {
        return new SlideTopBasket();
    }

    public class Retract implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of retracted position
                slidesLeftPID.setTargetPosition(0);
                slidesRightPID.setTargetPosition(0);
                init = true;
            }
            slidesLeftMotor.setPower(slidesLeftPID.update(slidesLeftMotor.getCurrentPosition()));
            slidesRightMotor.setPower(slidesRightPID.update(slidesRightMotor.getCurrentPosition()));

            if (Math.abs(slidesLeftPID.getTargetPosition() - getPos()) < 15) {
                return true;
            }
            return false;
        }
    }
    public Action retract() {
        return new Retract();
    }


    public double getPos() {
        return (double) (slidesLeftMotor.getCurrentPosition() + slidesRightMotor.getCurrentPosition()) / 2;
    }

//    public void slide(double pos) {
//        slidesLeftPID.setTargetPosition(pos);
//        slidesRightPID.setTargetPosition(pos);
//        moving = true;
//    }
//
//    public double getPos() {
//        return (double) (slidesLeftMotor.getCurrentPosition() + slidesRightMotor.getCurrentPosition()) / 2;
//    }
//
//    public void updatePID() {
//        slidesLeftPID.update(slidesLeftMotor.getCurrentPosition());
//        slidesRightPID.update(slidesRightMotor.getCurrentPosition());
//        if (Math.abs(slidesLeftPID.getTargetPosition() - getPos()) < 10) {
//            moving = false;
//        }
//    }
}
