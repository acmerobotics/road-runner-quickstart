package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 510 * LIFT_TICKS_PER_MM * 1.3;


    public DcMotorEx motor;
    double liftPosition;
    public Lift(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "armext");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPosition = LIFT_COLLAPSED;
    }

    // auto
    public class LiftAction implements Action {
        private final int _targetPos;
        // Constructor
        public LiftAction(int targetPos){
            _targetPos = targetPos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            motor.setTargetPosition(_targetPos);
            motor.setVelocity(3000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int currentPosition = motor.getCurrentPosition();
            int error = Math.abs(_targetPos - currentPosition);
            packet.put("LiftTargetPos", _targetPos);
            packet.put("LiftPos", currentPosition);
            packet.put("LiftError", error);
            // keep running until we're close enough
            return error > 5; // ticks
        }
    }

    public Action liftUpAction() {
        return new LiftAction((int) (LIFT_SCORING_IN_HIGH_BASKET));
    }

    public Action liftDownAction() {
        return new LiftAction((int) (LIFT_COLLAPSED));
    }


}
