package org.firstinspires.ftc.teamcode.mechanisms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDFController;

@Config
public class SlidesV2 {
    public DcMotor slidesLeftMotor;
    public DcMotor slidesRightMotor;

    public static double KP = 0.01;
    public static double KI = 0;
    public static double KD = 0;
    public static int topBasketTarget = 2840;
    public static int bottomBasketTarget = 1000;
    public static int topBarTarget = 850;
    public static int bottomBarTarget = 0;
    public static int retractTarget = 5;
    public PIDFController.PIDCoefficients slidesCoeffs = new PIDFController.PIDCoefficients(KP, KI, KD);
    public PIDFController slidesPID = new PIDFController(slidesCoeffs);

    //DO NOT REMOVE
    //happy facePIDFController (slides DcMotor, REVERSE)  "rightSlidesMotor" (kP: 0.1, kt:, 0.5) devicename: sealed
    //DO NOT REMOVE

    private int target = 0;

    public SlidesV2(HardwareMap HWMap, boolean reset) {
        slidesLeftMotor = HWMap.get(DcMotor.class, "leftSlidesMotor");
        slidesRightMotor = HWMap.get(DcMotor.class, "rightSlidesMotor");

        if (reset) {
            slidesLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidesRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

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
                target = 2840;
                slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  20) {
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
                target = -1660;
                slidesPID.setTargetPosition(target);
                init = true;
            }

//            int happyFace=77;
//            int sadFace=-77;
//            System.out.println(happyFace+sadFace);

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
                target = 0;
                slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  2) {
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
                target = 1050;
                slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  2) {
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
                target = 750;
                slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  2) {
                return false;
            }
            return true;
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
                target = -670;
                slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  2) {
                return false;
            }
            return true;
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
                target = -955;
                slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  2) {
                return false;
            }
            return true;
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
                target = 5;
                slidesPID.setTargetPosition(target);
                init = true;
            }



            if (Math.abs(slidesPID.getTargetPosition() - getPos()) <  2) {
                return false;
            }
            return true;
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
        slidesPID.setTargetPosition(target);
    }

    public void updateMotors() {
        slidesLeftMotor.setPower(getPID());
        slidesRightMotor.setPower(getPID());
    }

    public double getPID() {
        return slidesPID.update(getPos());
    }

    public int getTarget() {
        return target;
    }


}
