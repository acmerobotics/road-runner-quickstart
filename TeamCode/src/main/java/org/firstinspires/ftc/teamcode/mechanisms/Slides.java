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

    //private PIDFController.PIDCoefficients slidesCoeffs = new PIDFController.PIDCoefficients(0.1, 0.5, 0);
    //private PIDFController slidesPID = new PIDFController(slidesCoeffs);

    private int target = 0;

    public Slides(HardwareMap HWMap){
        slidesLeftMotor = HWMap.get(DcMotor.class, "leftSlidesMotor");
        slidesRightMotor = HWMap.get(DcMotor.class, "rightSlidesMotor");

        slidesLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slidesRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public Slides(HardwareMap HWMap, int i){
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
                //TODO: set value to the motor position of top basket
                target = -2790;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(-1);
                slidesRightMotor.setPower(-1);
                //slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) <  10) {
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
                target = -1660;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(0.7);
                slidesRightMotor.setPower(0.7);
                init = true;
            }


            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) < 2) {
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
                target = -324;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(0.7);
                slidesRightMotor.setPower(0.7);
                init = true;
            }



            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) < 2) {
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
                target = -1625;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(0.7);
                slidesRightMotor.setPower(0.7);
                init = true;
            }



            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) < 2) {
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
                target = -710;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(0.7);
                slidesRightMotor.setPower(0.7);

                init = true;
            }



            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) < 15) {
                return true;
            }
            return false;
        }
    }
    public Action slideBottomBar() {
        return new SlideBottomBar();
    }


    public class SlideHangLevel implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of a bit above the bottom speciman bar
                target = -670;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(0.7);
                slidesRightMotor.setPower(0.7);

                init = true;
            }



            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) < 15) {
                return true;
            }
            return false;
        }
    }
    public Action slideHangLevel() {
        return new SlideHangLevel();
    }

    public class SlideHang implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of a bit above the bottom speciman bar
                target = -955;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(0.7);
                slidesRightMotor.setPower(0.7);

                init = true;
            }



            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) < 15) {
                return true;
            }
            return false;
        }
    }
    public Action slideHang() {
        return new SlideHang();
    }

    public class Retract implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set values to the motor position of retracted position
                target = 10;
                slidesLeftMotor.setTargetPosition(target);
                slidesRightMotor.setTargetPosition(target);

                slidesLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slidesLeftMotor.setPower(0.7);
                slidesRightMotor.setPower(0.7);
                init = true;
            }



            if (Math.abs(slidesLeftMotor.getTargetPosition() - getPos()) < 2) {
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


    public void changeTarget(int change) {
        target += change;
        slidesLeftMotor.setTargetPosition(target);
        slidesRightMotor.setTargetPosition(target);
    }


}
