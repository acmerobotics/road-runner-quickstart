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
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM * 1.3;


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

    public class LiftUp implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                double beginTs = Actions.now();
                initialized = true;
                motor.setTargetPosition((int) (LIFT_SCORING_IN_HIGH_BASKET));
                motor.setVelocity(1300);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            double pos = motor.getCurrentPosition();
            packet.put("LiftPos", pos);
            if (pos < (int)(LIFT_SCORING_IN_HIGH_BASKET*0.98)) {
                motor.setTargetPosition((int) (LIFT_SCORING_IN_HIGH_BASKET));
                motor.setVelocity(1300);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }
    }

    public Action liftUpAction() {
        return new LiftUp();
    }

    public class LiftDown implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                double beginTs = Actions.now();
                initialized = true;
                motor.setTargetPosition((int) (LIFT_COLLAPSED));
                motor.setVelocity(1300);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            double pos = motor.getCurrentPosition();
            packet.put("LiftPos", pos);
            if (pos > (int)(20)) {
                motor.setTargetPosition((int) (LIFT_COLLAPSED));
                motor.setVelocity(1300);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }
    }

    public Action liftDownAction() {
        return new LiftDown();
    }

}
