package org.firstinspires.ftc.teamcode.mechanisms;



import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDFController;

public class Extendo {
    public DcMotor extendoMotor;

//    private PIDFController.PIDCoefficients extendoMotorCoeffs = new PIDFController.PIDCoefficients(0.12, 0, 0);
//    public PIDFController extendoMotorPID = new PIDFController(extendoMotorCoeffs);

    int target = -10;

    public Extendo(HardwareMap HWMap){
        extendoMotor = HWMap.get(DcMotor.class, "extendoMotor");
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public class Retract implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = -12;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendoMotor.setPower(0.5);
                //extendoMotorPID.setTargetPosition(extendo_target);
                init = true;
            }


            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 2) {
                return false;
            }
            return true;
        }
    }
    public Action retract() {
        return new Retract();
    }

    public class Extend implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                target = -55;
                extendoMotor.setTargetPosition(target);
                extendoMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendoMotor.setPower(-0.5);
                init = true;
            }


            if (Math.abs(extendoMotor.getTargetPosition() - getPos()) < 2) {
                return false;
            }
            return true;
        }
    }
    public Action extend() {
        return new Extend();
    }


    public double getPos() {
        return (extendoMotor.getCurrentPosition());
    }


    public void changetarget(double change){
        target += 0.5+change;
        extendoMotor.setTargetPosition(target);
    }
}


