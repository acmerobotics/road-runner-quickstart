package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {

    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
//define junction heights here//
    private final int STAGE_1_REVOLUTIONS = 1;
    private final int STAGE_2_REVOLUTIONS = 2;
    private final int STAGE_3_REVOLUTIONS = 3;

    //basically sets up robot//
    public Lift (DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //uses the defined junction heights to tell the motors to go to rotate till a certain height//
    public void liftToStage(int targetStage) {
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
        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);
    }
}

