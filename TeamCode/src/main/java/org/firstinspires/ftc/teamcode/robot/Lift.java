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
    public DcMotorEx liftMotor1;
    private Map stateMap;
    private PIDController liftController;
    //Create Strings for StateMap Lift Control
    public final String LIFT_SYSTEM_NAME = "LIFT_SYSTEM_NAME";
    public final String LIFT_GROUND_STATE = "LIFT_GROUND_STATE";
    public final String LIFT_LOW_STATE = "LIFT_LOW_STATE";
    public final String LIFT_MIDDLE_STATE = "LIFT_MIDDLE_STATE";
    public final String LIFT_HIGH_STATE = "LIFT_HIGH_STATE";

    public final String LIFT_IDLE_STATE = "LIFT_IDLE_STATE";

    public final String TRANSITION_STATE = "TRANSITION_STATE";

    public final String LIFT_CURRENT_STATE = "LIFT_CURRENT_STATE";

    // Encoder Positions for Lift heights
    private int LIFT_GROUND_STATE_POSITION = 0;
    private int LIFT_LOW_STATE_POSITION = 300;
    private int LIFT_MIDDLE_STATE_POSITION = 550;
    private int LIFT_HIGH_STATE_POSITION = 700;

    private int LIFT_IDLE_STATE_POSITION = 140;


    //Other important variables
    private int heightTolerance = 5;

    public Lift(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;


        liftController = new PIDController(0,0,0);
        liftMotor1 = hwMap.get(DcMotorEx.class, "liftMotor1");

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        liftController.setInputBounds(LIFT_GROUND_STATE_POSITION, LIFT_HIGH_STATE_POSITION);
//        liftController.setOutputBounds(0,1);
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
        int currentPosition = liftMotor1.getCurrentPosition();
        if(inHeightTolerance(currentPosition, LIFT_GROUND_STATE_POSITION)){
            state = LIFT_GROUND_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_LOW_STATE_POSITION)){
            state = LIFT_LOW_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_MIDDLE_STATE_POSITION)){
            state = LIFT_MIDDLE_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_HIGH_STATE_POSITION)){
            state = LIFT_HIGH_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_IDLE_STATE_POSITION)){
            state = LIFT_IDLE_STATE;
        }
        return state;
    }

    private boolean inHeightTolerance(int currentPosition, int statePosition){
        if((statePosition - heightTolerance) <= currentPosition && currentPosition <=  (statePosition + heightTolerance)){
            return true;
        }
        return false;
    }

    private void selectTransition(String desiredState){
        switch (desiredState){
            case LIFT_GROUND_STATE:{
                raiseHeightTo(LIFT_GROUND_STATE_POSITION);
                break;
            }
            case LIFT_LOW_STATE:{
                raiseHeightTo(LIFT_LOW_STATE_POSITION);
                break;
            }
            case LIFT_MIDDLE_STATE:{
                raiseHeightTo(LIFT_MIDDLE_STATE_POSITION);
                break;
            }
            case LIFT_HIGH_STATE:{
                raiseHeightTo(LIFT_HIGH_STATE_POSITION);
                break;
            }
            case LIFT_IDLE_STATE:{
                raiseHeightTo(LIFT_IDLE_STATE_POSITION);
                break;
            }
        }
    }

    private void raiseHeightTo(int desiredTickPosition){
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setTargetPosition(desiredTickPosition);
        liftMotor1.setPower(0.5);
    }
    private void moveToPID(int desiredTickPosition){
        int currentPosition = liftMotor1.getCurrentPosition();
        int error = Math.abs(currentPosition - desiredTickPosition);
        if(error < 7){
            liftMotor1.setTargetPosition(desiredTickPosition);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        liftMotor1.setPower(power);
    }
}
