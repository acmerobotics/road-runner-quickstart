package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;
public class Intake {
    private Telemetry telemetry;
    private Map stateMap;
    public DcMotorEx intakeMotor;
    private Constants constants = new Constants();
    private float gain = 10;

    public final String INTAKE_SYSTEM_NAME = "INTAKE_SYSTEM_NAME";
    public final String INTAKE_INTAKING_STATE = "INTAKE_INTAKING_STATE";
    public final String INTAKE_SPITTING_STATE = "INTAKE_NOT_INTAKING_STATE";
    public final String INTAKE_PIXEL_PICKUP_CYCLE = "INTAKE_PIXEL_PICKUP_CYCLE";
    public final String INTAKE_PIXEL_PICKUP_STATE_NOT_STARTED = "INTAKE_PIXEL_PICKUP_STATE_NOT_STARTED";
    public final String INTAKE_IDLE_STATE = "INTAKE_IDLE_STATE";
    public final String INTAKE_PIXEL_PICKUP_STATE_IN_PROGRESS = "INTAKE_PIXEL_PICKUP_STATE_IN_PROGRESS";
    public final String INTAKE_PIXEL_PICKUP_STATE_COMPLETED = "INTAKE_PIXEL_PICKUP_STATE_COMPLETED";

    private Hopper hopper;

    public Intake(HardwareMap hwMap, Telemetry telemetry, Map stateMap, Hopper hopper){
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        intakeMotor = hwMap.get(DcMotorEx.class, "IntakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.hopper = hopper;
    }

    public void setState(){
        String pixelString = (String)stateMap.get(constants.NUMBER_OF_PIXELS);
        int numOfPixels;
        if(pixelString.equalsIgnoreCase(constants.PIXEL_PICKUP_2_PIXELS)){
            numOfPixels = 2;
        } else{
            numOfPixels = 1;
        }
        telemetry.addData("cycle in progress",cycleInProgress());
        if(cycleInProgress()) {
            updatePixelPickupState(numOfPixels);
        }
        if(cycleInProgress()){
            intakeMotor.setPower(1.0);
        } else{
            intakeMotor.setPower(0.0);
        }
    }

    private void updatePixelPickupState(int numOfPixels){
        String hopperState = (String)stateMap.get(hopper.HOPPER_SYSTEM_NAME);
        if(hopperState.equalsIgnoreCase(hopper.HOPPER_ONE_PIXEL) && numOfPixels == 1){
            stateMap.put(constants.PIXEL_CYCLE_INTAKE, constants.PIXEL_CYCLE_STATE_COMPLETE);
        }
        if(hopperState.equalsIgnoreCase(hopper.HOPPER_TWO_PIXELS) && numOfPixels == 2) {
            stateMap.put(constants.PIXEL_CYCLE_INTAKE, constants.PIXEL_CYCLE_STATE_COMPLETE);
        }
    }

    private boolean cycleInProgress(){
        String intakeCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE);
        if(intakeCycleState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            telemetry.addLine("Cycle was true");
        }
        return intakeCycleState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
    }

    private boolean intakeShouldMove(){
        String intakeState = (String)stateMap.get(INTAKE_SYSTEM_NAME);
        String pixelState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE);
        if(pixelState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS) || intakeState.equals(INTAKE_SPITTING_STATE)){
            return true;
        }
        return false;
    }

    private void selectTransition(){
        String state = (String) stateMap.get(INTAKE_SYSTEM_NAME);
        String pixelCycleIntake = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE);
        switch(state){
            case INTAKE_PIXEL_PICKUP_STATE_IN_PROGRESS:{
                intakeMotor.setPower(1.0);
            }
            case INTAKE_SPITTING_STATE:{
                intakeMotor.setPower(1.0);
            }
        }
    }
}
