package org.firstinspires.ftc.teamcode.mechanisms.robotv2;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.PublicKey;

@Config
public class LeftActuator {
    public DcMotorEx motor;

    public static int ACTUATOR_COLLAPSED = 0;
    public static int ACTUATOR_UP = 1150;

    public LeftActuator(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "leftactuator");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // call reset() only from autonomous code and not from the teleop.
    // This way, the arm encoder position is not reset between autonomous and teleop.
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
    }

    public void up() {
        motor.setTargetPosition(ACTUATOR_UP);
    }
    public void down() {
        motor.setTargetPosition(ACTUATOR_COLLAPSED);
    }
}

