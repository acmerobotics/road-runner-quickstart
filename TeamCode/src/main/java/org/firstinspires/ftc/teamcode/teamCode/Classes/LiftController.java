package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utils.PIDController;

public class LiftController { //clasa veche lift controller
    DcMotorEx left, right;
    //double kp = 0.02, kd = 0.01, ki = 0.002;
    double kp = 0.035, kd = 0.05, ki = 0.5;
    PIDController pidController = new PIDController(kp, kd, ki);
    double magicPOWER = 0.3;
    int PLACE_POS = 900;
    public int position;
    int SAFE_POS = 400;
    public static int MAX_POS = 2200;
    public static int UP_POS = 2200;
    public static int HANG_POS = 2200;
    boolean pidON = true;

    public enum States {
        GOING_DOWN_PID,
        GOING_DOWN_MAGIC,
        DOWN,
        UP
    }

    public States currentState = States.DOWN;

    ElapsedTime timer;
    int time_to_wait;

    public LiftController(HardwareMap map) {
        left = map.get(DcMotorEx.class, "leftLift");
        right = map.get(DcMotorEx.class, "rightLift");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setCurrentAlert(4, CurrentUnit.AMPS);
        right.setCurrentAlert(4, CurrentUnit.AMPS);

        pidController.targetValue = -10;

        timer = new ElapsedTime();
    }

    public void goingUp()
    {
        if (position < MAX_POS)
            pidController.targetValue = position + 10;
    }

    public void goingDown()
    {
        if(position > 10)
            pidController.targetValue = position - 10;
    }

    public void goToMid()
    {
        pidController.targetValue = PLACE_POS;
    }
    public void goToMax()
    {
        pidController.targetValue = UP_POS;
    }
    public void goToHang() {
        pidController.targetValue = HANG_POS;
    }
    public void goTo0() {
        pidController.targetValue = 0;
        currentState = States.GOING_DOWN_PID;
    }

    public void update() {

        switch (currentState)
        {
            case GOING_DOWN_PID:
            {
                if(Math.abs( position - 0) < 30)
                {
                    pidON = false;
                    currentState = States.GOING_DOWN_MAGIC;
                    left.setPower(magicPOWER);
                    right.setPower(magicPOWER);
                }
                break;
            }
            case GOING_DOWN_MAGIC:
            {
                if(left.isOverCurrent() || right.isOverCurrent())
                {
                    ResetEncoders();
                    pidController.targetValue = 0;
                    pidON = true;
                    currentState = States.DOWN;
                }
            }
        }

        if(pidON)
        {
            position = left.getCurrentPosition();
            //pidController.targetValue = rowPosition[targetRow];
            double powerLift = pidController.update(position);
            left.setPower(powerLift);
            right.setPower(powerLift);
        }

    }


    public void ResetEncoders() {
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
