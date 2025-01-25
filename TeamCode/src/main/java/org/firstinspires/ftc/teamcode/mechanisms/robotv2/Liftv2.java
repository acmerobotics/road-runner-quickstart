package org.firstinspires.ftc.teamcode.mechanisms.robotv2;




import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Liftv2 {
    public static  double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    public static double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    public static  double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    public static  double LIFT_SCORING_IN_HIGH_BASKET = 3550;

    public  static int LIFT_HANG_SLIDES_POSITION = 3200;

    public static int LIFT_HANG_SLIDES_POSITION_END = 2000;

    public static int LIFT_OUT_PICKUP_GROUND = 1500;

    public DcMotorEx motor;
    double liftPosition;
    public Liftv2(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftPosition = LIFT_COLLAPSED;
    }

    public void reset(){
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            motor.setVelocity(10000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int currentPosition = motor.getCurrentPosition();
            int error = Math.abs(_targetPos - currentPosition);
            packet.put("LiftTargetPos", _targetPos);
            packet.put("LiftPos", currentPosition);
            packet.put("LiftError", error);
            // keep running until we're close enough
            return error > 10; // ticks
        }
    }


    public Action liftUpAction() {
        return new LiftAction((int) (LIFT_SCORING_IN_HIGH_BASKET));
    }

    public Action liftDownAction() {
        return new LiftAction((int) (LIFT_COLLAPSED));
    }

    public Action liftOutPickUpAction() {
        return new LiftAction((int) (LIFT_OUT_PICKUP_GROUND));
    }



}

