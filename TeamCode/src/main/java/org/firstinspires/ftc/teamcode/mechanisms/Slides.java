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

    private PIDFController.PIDCoefficients slidesCoeffs = new PIDFController.PIDCoefficients(0.05, 0.5, 0);
    private PIDFController slidesPID = new PIDFController(slidesCoeffs);

    private double target = 0;

    public Slides(HardwareMap HWMap){
        slidesLeftMotor = HWMap.get(DcMotor.class, "leftSlidesMotor");
        slidesRightMotor = HWMap.get(DcMotor.class, "rightSlidesMotor");

        slidesLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slidesRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class SlideTopBasket implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set value to the motor position of top basket
                target = -2650;
                slidesPID.setTargetPosition(target);
                init = true;
            }

            updateMotor();

            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  2) {
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
                target = 100;
                slidesPID.setTargetPosition(target);
                init = true;
            }

            updateMotor();

            if (Math.abs(slidesPID.getTargetPosition() - getPos()) < 2) {
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
                target = 200;
                slidesPID.setTargetPosition(target);
                init = true;
            }

            updateMotor();

            if (Math.abs(slidesPID.getTargetPosition() - getPos()) < 2) {
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
                target = 250;
                slidesPID.setTargetPosition(target);
                init = true;
            }

            updateMotor();

            if (Math.abs(slidesPID.getTargetPosition() - getPos()) < 2) {
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
                target = 250;
                slidesPID.setTargetPosition(target);
                init = true;
            }

            updateMotor();

            if (Math.abs(slidesPID.getTargetPosition() - getPos()) < 15) {
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
                target = 0;
                slidesPID.setTargetPosition(0);
                init = true;
            }

            updateMotor();

            if (Math.abs(slidesPID.getTargetPosition() - getPos()) < 2) {
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

    public void updateMotor() {
        slidesLeftMotor.setPower(slidesPID.update(getPos()));
        slidesRightMotor.setPower(slidesPID.update(getPos()));
    }

    public void changeTarget(double change) {
        target += change;
        slidesPID.setTargetPosition(target);
    }

}
