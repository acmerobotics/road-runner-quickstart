package org.firstinspires.ftc.teamcode.mechanisms.v1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Arm {
    public DcMotorEx motor;

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    public static double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 71.2 // Gear ratio for the for the motor used in V1 robot
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    public static double ARM_COLLECT_DEG = 4;
    public static double ARM_COLLAPSED_INTO_ROBOT  = 0 * ARM_TICKS_PER_DEGREE;
    public static double ARM_COLLECT               = ARM_COLLECT_DEG * ARM_TICKS_PER_DEGREE;
    public static double ARM_CLEAR_BARRIER         = 19 * ARM_TICKS_PER_DEGREE;
    public static double ARM_SCORE_SPECIMEN        = 138 * ARM_TICKS_PER_DEGREE;
    public static double ARM_SCORE_SAMPLE_IN_LOW   = 138 * ARM_TICKS_PER_DEGREE;
    public static double ARM_SCORE_SAMPLE_IN_HIGH   = 128 * ARM_TICKS_PER_DEGREE;
    public static double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    public static double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    public static double ARM_PARK_POS = 60 * ARM_TICKS_PER_DEGREE;

    public static double ARM_VERTICAL = 120 * ARM_TICKS_PER_DEGREE;
    public static double ARM_ROBOT_TRAVEL = 219 * ARM_TICKS_PER_DEGREE;

    /* A number in degrees that the triggers can adjust the arm position by */
    public static double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;


    public Arm(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // call reset() only from autonomous code and not from the teleop.
    // This way, the arm encoder position is not reset between autonomous and teleop.
    public void reset(){
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // auto
    public class ArmScoreAuto implements Action {
        private final int _targetPos;
        // Constructor
        public ArmScoreAuto(int targetPos){
            _targetPos = targetPos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            motor.setTargetPosition(_targetPos);
            motor.setVelocity(2000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int currentPosition = motor.getCurrentPosition();
            int error = Math.abs(_targetPos - currentPosition);
            packet.put("ArmTarget", _targetPos);
            packet.put("ArmPos", currentPosition);
            packet.put("ArmError", error);
            // keep running until we're close enough
            return error > 5; // ticks
        }
    }
    public Action armScoreAction() {
        return new ArmScoreAuto((int)ARM_SCORE_SAMPLE_IN_HIGH);
    }
    public Action armfoldbackaction() {
        return new ArmScoreAuto((int)ARM_COLLAPSED_INTO_ROBOT);
    }
    public Action armGroundCollectAction(){
        return new ArmScoreAuto((int)ARM_COLLECT);
    }
    public Action armRobotTravelAction(){
        return new ArmScoreAuto((int)ARM_ROBOT_TRAVEL);
    }
    public Action armVerticalAction(){
        return new ArmScoreAuto((int)ARM_VERTICAL);
    }

    public Action armParkAction(){return  new ArmScoreAuto((int)ARM_PARK_POS);

    }
}