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

import java.security.PublicKey;

@Config
public class Armv2 {
    public DcMotorEx motor;

    public static int ARM_REST_POSITION = 0;
    public static int ARM_VERTICAL = 0;

    public static int ARM_COME_DOWN = 450;

    public static int ARM_CLEAR_BAR = 1200;
    public static int ARM_CLEAR_BAR_LIFT_OUT = 1200;

    public static int ARM_SCORE_POS = 180;
    public static int ARM_PICKUP_GROUND_SAMPLE = 1300;
    public static int ARM_PICKUP_GROUND_SAMPLE_LIFT_OUT = 1390;

    public static int ARM_HANG_SLIDES_POSITION = 0;

    /* A number in degrees that the triggers can adjust the arm position by */
    public static double FUDGE_FACTOR = 300;
    public static double ARM_VELOCITY = 1000;
    public static double ARM_VELOCITY_SF = 1.0;
    public static double ARM_ACTION_TIMEOUT_SEC = 5.0;


    public Armv2(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // call reset() only from autonomous code and not from the teleop.
    // This way, the arm encoder position is not reset between autonomous and teleop.
    public void reset(){
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void pickupGroundSample(){
        motor.setTargetPosition(ARM_PICKUP_GROUND_SAMPLE);
    }

    // auto
    public class ArmScoreAuto implements Action {
        private final int _targetPos;
        private boolean initialized = false;
        private double beginTs = -1;
        // Constructor
        public ArmScoreAuto(int targetPos){
            _targetPos = targetPos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            if (!initialized){
                beginTs = Actions.now();
                initialized = true;
            }
            int currentPosition = motor.getCurrentPosition();
            int error = Math.abs(_targetPos - currentPosition);
            duration = Actions.now() - beginTs;

            motor.setTargetPosition(_targetPos);
            if (error > 50) {
                motor.setVelocity(ARM_VELOCITY);
            } else {
                motor.setVelocity(ARM_VELOCITY * ARM_VELOCITY_SF);
            }
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            packet.put("ArmTarget", _targetPos);
            packet.put("ArmPos", currentPosition);
            packet.put("ArmError", error);
            // keep running until we're close enough or we timeout
            return error > 5 && duration < ARM_ACTION_TIMEOUT_SEC;
        }
    }
    public Action armResetAction() {
        return new ArmScoreAuto(ARM_REST_POSITION);
    }

    public Action armVerticalAction()
    {
        return new ArmScoreAuto(ARM_VERTICAL);
    }
    public Action armScoreAction()
    {
        return new ArmScoreAuto(ARM_SCORE_POS);
    }
    public Action armPickupGroundSampleAction() {
        return new ArmScoreAuto(ARM_PICKUP_GROUND_SAMPLE);
    }
    public Action armPickupGroundSampleLiftOutAction() {
        return new ArmScoreAuto(ARM_PICKUP_GROUND_SAMPLE_LIFT_OUT);
    }
    public Action armClearBarLiftOutAction() {
        return new ArmScoreAuto(ARM_CLEAR_BAR_LIFT_OUT);
    }

    public Action armClearBarAction() {
        return new ArmScoreAuto(ARM_CLEAR_BAR);
    }
    public Action armComeDownAction()
    {
        return new ArmScoreAuto(ARM_COME_DOWN);
    }
}
