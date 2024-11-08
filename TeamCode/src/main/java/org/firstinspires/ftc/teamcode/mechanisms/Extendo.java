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

    private PIDFController.PIDCoefficients extendoMotorCoeffs = new PIDFController.PIDCoefficients(0.12, 0, 0);
    public PIDFController extendoMotorPID = new PIDFController(extendoMotorCoeffs);

    double extendo_target = -6;

    public Extendo(HardwareMap HWMap){
        extendoMotor = HWMap.get(DcMotor.class, "extendoMotor");
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public class Retract implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                extendo_target = -12;
                extendoMotorPID.setTargetPosition(extendo_target);
                init = true;
            }

            updateMotor();

            if (Math.abs(extendoMotorPID.getTargetPosition() - getPos()) < 2) {
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
                extendo_target = -65;
                extendoMotorPID.setTargetPosition(extendo_target);
                init = true;
            }

            updateMotor();

            if (Math.abs(extendoMotorPID.getTargetPosition() - getPos()) < 2) {
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

    public void updateMotor() {
        extendoMotor.setPower(extendoMotorPID.update(extendoMotor.getCurrentPosition()));
    }

    public void changetarget(double change){
        extendo_target += change;
        extendoMotorPID.setTargetPosition(extendo_target);
    }
}


