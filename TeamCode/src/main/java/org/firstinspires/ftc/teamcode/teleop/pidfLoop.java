package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class pidfLoop {
    public static double fP,fI, fD;
    public static double sP,sI,sD;
    private PIDController flipController = new PIDController(fP,fI, fD);
    private PIDController slidesController = new PIDController(sP, sI, sD);
    public static double fF;

    private final double ticks_in_degree = 100/180.0;
    private final double slidesFF = 0.1;

    public Action getPivotLoop(DcMotor arm, double target) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                flipController.setPID(fP,fI, fD);

                int armPos = arm.getCurrentPosition();
                double pid = flipController.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * fF;

                double power = pid + ff;

                arm.setPower(power);
                return false;
            }
        };
    }

    public Action getSlidesLoop(DcMotor slide, double target) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slidesController.setPID(sP,sI, sD);

                int armPos = slide.getCurrentPosition();
                double pid = slidesController.calculate(armPos, target);

                double power = pid + slidesFF;

                slide.setPower(power);
                return false;
            }
        };
    }
}
