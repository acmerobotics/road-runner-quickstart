package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang {


    final double HANG_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double HANG_COLLAPSED = 0 * HANG_TICKS_PER_MM;
    final double HANG_SCORING_IN_LOW_BASKET = 0 * HANG_TICKS_PER_MM;
    final double HANG_SCORING_IN_HIGH_BASKET = 510 * HANG_TICKS_PER_MM * 1.3;


    public DcMotorEx leftmotor;
    public DcMotorEx rightmotor;
    double hangPosition;

    public Hang(HardwareMap hardwareMap) {
        leftmotor = hardwareMap.get(DcMotorEx.class, "leftactuator");
        leftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftmotor.setTargetPosition(0);
        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPosition = HANG_COLLAPSED;

        rightmotor = hardwareMap.get(DcMotorEx.class, "rightactuator");
        rightmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setTargetPosition(0);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPosition = HANG_COLLAPSED;
    }



        // auto
        public class HangAction implements Action {
            private final int _targetPos;
            // Constructor
            public HangAction(int targetPos){
                _targetPos = targetPos;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                leftmotor.setTargetPosition(_targetPos);
                leftmotor.setVelocity(3000);
                leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                int currentPosition = leftmotor.getCurrentPosition();
                int error = Math.abs(_targetPos - currentPosition);
                packet.put("HangTargetPos", _targetPos);
                packet.put("HangPos", currentPosition);
                packet.put("HangError", error);
                // keep running until we're close enough
                return error > 5; // ticks
            }
        }

        public Action HangUpAction() {
            return new HangAction((int) (HANG_SCORING_IN_HIGH_BASKET));
        }

        public Action HangDownAction() {
            return new HangAction((int) (HANG_COLLAPSED));
        }


    }


