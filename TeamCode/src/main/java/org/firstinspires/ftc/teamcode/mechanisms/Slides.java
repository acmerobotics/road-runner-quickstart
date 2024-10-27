package org.firstinspires.ftc.teamcode.mechanisms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDFController;

public class Slides {
    public DcMotor slidesLeftMotor;
    public DcMotor slidesRightMotor;

    private PIDFController.PIDCoefficients slidesLeftCoeffs = new PIDFController.PIDCoefficients(1, 0 , 0);
    private PIDFController.PIDCoefficients slidesRightCoeffs = new PIDFController.PIDCoefficients(1, 0 , 0);
    private PIDFController slidesLeftPID = new PIDFController(slidesLeftCoeffs);
    private PIDFController slidesRightPID = new PIDFController(slidesRightCoeffs);


    public Slides(HardwareMap HWMap){
        slidesLeftMotor = HWMap.get(DcMotor.class, "leftSlidesMotor");
        slidesRightMotor = HWMap.get(DcMotor.class, "rightSlidesMotor");

        slidesLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slidesRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
                return false;
            }
            return true;
        }
    }
    public Action slideTopBasket() {
        return new SlideTopBasket();
    }

    public class SlideBottomBasket implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of bottom basket
                slidesLeftPID.setTargetPosition(100);
                slidesRightPID.setTargetPosition(100);
                init = true;
            }
            slidesLeftMotor.setPower(slidesLeftPID.update(slidesLeftMotor.getCurrentPosition()));
            slidesRightMotor.setPower(slidesRightPID.update(slidesRightMotor.getCurrentPosition()));

            if (Math.abs(slidesLeftPID.getTargetPosition() - getPos()) < 15) {
                return false;
            }
            return true;
        }
    }
    public Action slideBottomBasket() {
        return new SlideBottomBasket();
    }

    public class SlideWallLevel implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of the wall
                slidesLeftPID.setTargetPosition(200);
                slidesRightPID.setTargetPosition(200);
                init = true;
            }
            slidesLeftMotor.setPower(slidesLeftPID.update(slidesLeftMotor.getCurrentPosition()));
            slidesRightMotor.setPower(slidesRightPID.update(slidesRightMotor.getCurrentPosition()));

            if (Math.abs(slidesLeftPID.getTargetPosition() - getPos()) < 15) {
                return false;
            }
            return true;
        }
    }
    public Action slideWallLevel() {
        return new SlideWallLevel();
    }

    public class SlideTopBar implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of a bit above the top speciman bar
                slidesLeftPID.setTargetPosition(250);
                slidesRightPID.setTargetPosition(250);
                init = true;
            }
            slidesLeftMotor.setPower(slidesLeftPID.update(slidesLeftMotor.getCurrentPosition()));
            slidesRightMotor.setPower(slidesRightPID.update(slidesRightMotor.getCurrentPosition()));

            if (Math.abs(slidesLeftPID.getTargetPosition() - getPos()) < 15) {
                return false;
            }
            return true;
        }
    }
    public Action slideTopBar() {
        return new SlideTopBar();
    }

    public class SlideBottomBar implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of a bit above the bottom speciman bar
                slidesLeftPID.setTargetPosition(250);
                slidesRightPID.setTargetPosition(250);
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
    public Action slideBottomBar() {
        return new SlideBottomBar();
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
}
