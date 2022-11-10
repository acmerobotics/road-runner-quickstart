package org.firstinspires.ftc.teamcode.lift;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

//define junction heights here//
    private static final int LOW_JUNCTION_ENCODER_POSITION = 1300;
    private static final int MID_JUNCTION_ENCODER_POSITION = 2100;
    private static final int HIGH_JUNCTION_ENCODER_POSITION = 0;

    private static final int CONE_ENCODER_POSITION = 115;

    private static final double REST_POWER = 0.0025;
    private static final double LIFT_POWER = 0.3;

    //basically sets up robot//
    public Lift (final DcMotor leftMotor, final DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        useZeroPowerBehavior(zeroPowerBehavior);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        useRunMode(runMode);
    }

    public void useRunMode(final DcMotor.RunMode runMode) {
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
        this.runMode = runMode;
    }

    public void useZeroPowerBehavior(final DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    public void reset() {
        useZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void resetEncoder() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoder() {

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void useJoystick(final double joystickWeight) {
        if (runMode != DcMotor.RunMode.RUN_USING_ENCODER)
            useRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (joystickWeight == 0) {
            leftMotor.setPower(REST_POWER);
            rightMotor.setPower(REST_POWER);
        } else {
            if (leftMotor.getCurrentPosition()<2) {
                leftMotor.setPower(Math.abs(joystickWeight));
                rightMotor.setPower(Math.abs(-joystickWeight));
            }   else {
                leftMotor.setPower(joystickWeight);
                rightMotor.setPower(joystickWeight);
            }
        }
    }

    //uses the defined junction heights to tell the motors to go to rotate till a certain height//
    public void liftToJunction(final int targetJunction) {
        switch (targetJunction) {
            case 1:
                leftMotor.setTargetPosition(LOW_JUNCTION_ENCODER_POSITION);
                rightMotor.setTargetPosition(LOW_JUNCTION_ENCODER_POSITION);
                break;
            case 2:
                leftMotor.setTargetPosition(MID_JUNCTION_ENCODER_POSITION);
                rightMotor.setTargetPosition(MID_JUNCTION_ENCODER_POSITION);
                break;
            case 3:
                leftMotor.setTargetPosition(HIGH_JUNCTION_ENCODER_POSITION);
                rightMotor.setTargetPosition(HIGH_JUNCTION_ENCODER_POSITION);
        }

        useRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(LIFT_POWER);
        rightMotor.setPower(LIFT_POWER);
    }

    public void liftToConeStack(final int coneStackHeight) {
        leftMotor.setTargetPosition(CONE_ENCODER_POSITION * coneStackHeight);
        rightMotor.setTargetPosition(CONE_ENCODER_POSITION * coneStackHeight);

        useRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(LIFT_POWER);
        rightMotor.setPower(LIFT_POWER);
    }

    //resets the motors//
    public void retract() {
        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);

        useRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(LIFT_POWER);
        rightMotor.setPower(LIFT_POWER);
    }

    public double[] getPower() {
        return new double[] {leftMotor.getPower(), rightMotor.getPower()};
    }
}

