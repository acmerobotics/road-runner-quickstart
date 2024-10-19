package org.firstinspires.ftc.teamcode.mechanisms;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDFController;

public class Extendo {
    public DcMotor extendoMotor;

    public PIDFController.PIDCoefficients extendoMotorCoeffs = new PIDFController.PIDCoefficients(1, 0 , 0);
    public PIDFController extendoMotorPID = new PIDFController(extendoMotorCoeffs);

    //public boolean moving = false;
    public Extendo(HardwareMap HWMap){
        extendoMotor = HWMap.get(DcMotor.class, "extendoMotor");

        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public class Retract implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //TODO: set value to retracted extendo position
                extendoMotorPID.setTargetPosition(0);
                init = true;
            }

            extendoMotor.setPower(extendoMotorPID.update(extendoMotor.getCurrentPosition()));

            if (Math.abs(extendoMotorPID.getTargetPosition() - getPos()) < 15) {
                Intake.flipped = false;
                return true;
            }
            return false;
        }
    }
    public Action retract() {
        return new Retract();
    }

    public double getPos() {
        return (double) (extendoMotor.getCurrentPosition()) / 2;
    }
}


