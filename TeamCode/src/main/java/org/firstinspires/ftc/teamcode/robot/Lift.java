package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;
public class Lift {
    private Telemetry telemetry;
    private DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;
    private Map stateMap;
    private PIDController liftController;
    //Create Strings for StateMap Lift Control
    public final String LIFT_SYSTEM_NAME = "LIFT_SYSTEM";
    public final String LIFT_GROUND_STATE = "LIFT_GROUND_STATE";
    public final String LIFT_LOW_STATE = "LIFT_LOW_STATE";
    public final String LIFT_MIDDLE_STATE = "LIFT_MIDDLE_STATE";
    public final String LIFT_HIGH_STATE = "LIFT_HIGH_STATE";
    public final String TRANSITION_STATE = "TRANSITION_STATE";

    public final String LIFT_CURRENT_STATE = "LIFT_CURRENT_STATE";

    // Encoder Positions for Lift heights
    private int LIFT_GROUND_STATE_POSITION = 0;
    private int LIFT_LOW_STATE_POSITION;
    private int LIFT_MIDDLE_STATE_POSITION;
    private int LIFT_HIGH_STATE_POSITION;

    //Other important variables
    private int heightTolerance = 5;

    public Lift(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        liftController = new PIDController(0,0,0);
        liftMotor1 = hwMap.get(DcMotorEx.class, "LiftMotor1");
        liftMotor2 = hwMap.get(DcMotorEx.class, "LiftMotor2");

        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftController.setInputBounds(LIFT_GROUND_STATE_POSITION, LIFT_HIGH_STATE_POSITION);
        liftController.setOutputBounds(0,1);
    }

    public int getAvgPosition(){
        int avgPosition = (liftMotor1.getCurrentPosition() + liftMotor2.getCurrentPosition())/2;
        telemetry.addData("Lift Average", avgPosition);
        return avgPosition;
    }

    public void setState(){
        String currentLevel = getCurrentState();

        String desiredLevel = (String) stateMap.get(LIFT_SYSTEM_NAME);

        stateMap.put(LIFT_CURRENT_STATE, currentLevel);

        if(desiredLevel.equalsIgnoreCase(currentLevel)) {
            setRawPower(0);
        } else {
            selectTransition(desiredLevel);
        }
    }

    private String getCurrentState() {
        String state = TRANSITION_STATE;
        int currentPosition = getAvgPosition();
        if(inHeightTolerance(currentPosition,LIFT_GROUND_STATE_POSITION)){
            state = LIFT_GROUND_STATE;
        } else if(inHeightTolerance(currentPosition,LIFT_LOW_STATE_POSITION)){
            state = LIFT_LOW_STATE;
        } else if(inHeightTolerance(currentPosition,LIFT_MIDDLE_STATE_POSITION)){
            state = LIFT_MIDDLE_STATE;
        } else if(inHeightTolerance(currentPosition,LIFT_HIGH_STATE_POSITION)){
            state = LIFT_HIGH_STATE;
        }
        return state;
    }

    private boolean inHeightTolerance(int currentPosition, int statePosition){
        if(statePosition - heightTolerance <= currentPosition && currentPosition >=  statePosition + heightTolerance){
            return true;
        }
        return false;
    }

    private void selectTransition(String desiredState){
        switch (desiredState){
            case LIFT_GROUND_STATE:{
                moveToPID(LIFT_GROUND_STATE_POSITION);
            }
            case LIFT_LOW_STATE:{
                moveToPID(LIFT_LOW_STATE_POSITION);
            }
            case LIFT_MIDDLE_STATE:{
                moveToPID(LIFT_MIDDLE_STATE_POSITION);
            }
            case LIFT_HIGH_STATE:{
                moveToPID(LIFT_HIGH_STATE_POSITION);
            }
        }
    }

    private void moveToPID(int desiredTickPosition){
        int currentPosition = getAvgPosition();
        int error = Math.abs(currentPosition - desiredTickPosition);
        if(error < 7){
            liftMotor1.setTargetPosition(desiredTickPosition);
            liftMotor2.setTargetPosition(desiredTickPosition);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setRawPower(1.0);
        }else{
            setMotorPIDPower(desiredTickPosition, currentPosition);
        }
    }

    private void setMotorPIDPower(int ticks, int currentPosition){
        if(ticks != liftController.getTarget()) {
            liftController.reset();
            liftController.setTarget(ticks);
        }
        double power = (liftController.update(currentPosition));
        if(currentPosition > ticks) {
            setRawPower(-power);
        }else{
            setRawPower(power);
        }
    }

    public void setRawPower(double power) {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setPower(power);
        liftMotor1.setPower(power);
    }
}
