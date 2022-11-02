package org.firstinspires.ftc.teamcode.lift;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

//define junction heights here//
    private static final int STAGE_1_REVOLUTIONS = 1;
    private static final int STAGE_2_REVOLUTIONS = 2;
    private static final int STAGE_3_REVOLUTIONS = 3;

    //basically sets up robot//
    public Lift (final DcMotor leftMotor, final DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        useZeroPowerBehavior(zeroPowerBehavior);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public void useJoystick(final double joystickWeight) {
        if (runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            useRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (joystickWeight > 0) {
            leftMotor.setPower(joystickWeight);
            rightMotor.setPower(-joystickWeight);
        } else if(joystickWeight < 0) {
            leftMotor.setPower(-joystickWeight);
            rightMotor.setPower(joystickWeight);
        }
    }

    //uses the defined junction heights to tell the motors to go to rotate till a certain height//
    public void liftToStage(int targetStage) {
        if (this.runMode != DcMotor.RunMode.RUN_TO_POSITION)
            useRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        switch (targetStage) {
            case 1:
                leftMotor.setTargetPosition(STAGE_1_REVOLUTIONS);
                rightMotor.setTargetPosition(STAGE_1_REVOLUTIONS);
            case 2:
                leftMotor.setTargetPosition(STAGE_2_REVOLUTIONS);
                rightMotor.setTargetPosition(STAGE_2_REVOLUTIONS);
            case 3:
                leftMotor.setTargetPosition(STAGE_3_REVOLUTIONS);
                rightMotor.setTargetPosition(STAGE_3_REVOLUTIONS);
        }
    }
    //resets the motors//
    public void retract() {
        if (this.runMode != DcMotor.RunMode.RUN_TO_POSITION)
            useRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);
    }

    public double[] getPower() {
        return new double[] {leftMotor.getPower(), rightMotor.getPower()};
    }
}

