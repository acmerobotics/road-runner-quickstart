package org.firstinspires.ftc.teamcode.mechanisms;



import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDFController;

public class Extendo {
    public DcMotor extendoMotor;

    private int target = 0;

    public Extendo(HardwareMap HWMap){
        extendoMotor = HWMap.get(DcMotor.class, "extendoMotor");
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public Extendo(HardwareMap HWMap, int i) {
        extendoMotor = HWMap.get(DcMotor.class, "extendoMotor");
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class Retract implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = 0;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendoMotor.setPower(1);
                //extendoMotorPID.setTargetPosition(target);
                init = true;
            }

            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 5) {
                extendoMotor.setPower(0);
                return false;
            }
            return true;
        }
    }
    public Action retract() {
        return new Retract();
    }

    public class RetractS implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = -10;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendoMotor.setPower(1);
//                extendoMotorPID.setTargetPosition(target);
                init = true;
            }


            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 2) {
                extendoMotor.setPower(0);
                return false;
            }
            return true;
        }
    }
    public Action retractS() {
        return new RetractS();
    }


    public class Extend implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = -80;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendoMotor.setPower(1);
//                extendoMotorPID.setTargetPosition(target);
                init = true;
            }


            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 5) {
                return false;
            }
            return true;
        }
    }
    public Action extend() {
        return new Extend();
    }


    public class ExtendBad implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = -100;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendoMotor.setPower(1);
//                extendoMotorPID.setTargetPosition(target);
                init = true;
            }


            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 5) {
                return false;
            }
            return true;
            /*

            if (!init) {
                //pid = false;
                extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                target = -90;
                extendoMotor.setPower(-0.3);
                init = true;
            }

            if (Math.abs(-90 - getPos()) < 2) {
                //pid = true;
                extendoMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
            return true;
            */

        }
    }

    public Action extendBad() {
        return new ExtendBad();
    }


    public class Mid implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = -39;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendoMotor.setPower(1);
//                extendoMotorPID.setTargetPosition(target);
                init = true;
            }


            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 3) {
                return false;
            }
            return true;
        }
    }
    public Action mid() {
        return new Mid();
    }

    public class Mid2 implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = -39;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendoMotor.setPower(1);
//                extendoMotorPID.setTargetPosition(target);
                init = true;
            }


            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 3) {
                return false;
            }
            return true;
        }
    }
    public Action mid2() {
        return new Mid2();
    }


    public double getPos() {
        return (extendoMotor.getCurrentPosition());
    }
//
//    public void updateMotor() {
//        extendoMotor.setPower(extendoMotorPID.update(extendoMotor.getCurrentPosition()));
//    }

    public void changetarget(double change){
        target += change;
        extendoMotor.setTargetPosition(target);
    }

    public void setTarget(int pos) {
        extendoMotor.setTargetPosition(pos);
    }


    public void setManual() {
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setAuto() {
        extendoMotor.setTargetPosition(target);
        extendoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}


