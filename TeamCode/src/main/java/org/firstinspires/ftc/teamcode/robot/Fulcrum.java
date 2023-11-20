package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Fulcrum {
    private Telemetry telemetry;
    private Map stateMap;
    private ServoImplEx fulcrumServo;
    private final double down = 1.0;
    private final double up = 0;
    private double lowerPWMLimit = 352;
    private double upperPWMLimit = 1070;
    public ElapsedTime fulcrumCycleTime = new ElapsedTime();

    private Constants constants = new Constants();

    //Statemap strings
    public final String FULCRUM_SYSTEM_NAME = "FULCRUM_SYSTEM_NAME";
    public final String FULCRUM_DOWN = "FULCRUM_DOWN";
    public final String FULCRUM_UP = "FULCRUM_UP";
    public Fulcrum(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap =  stateMap;
        fulcrumServo = hwMap.get(ServoImplEx.class, "fulcrumServo");

        fulcrumServo.setPwmRange(new PwmControl.PwmRange(lowerPWMLimit,upperPWMLimit));

    }

    public void setState(){
        telemetry.addData("Fulcrum cycle in progress", cycleIsInProgress());
        if(cycleIsInProgress()){
            stateMap.put(FULCRUM_SYSTEM_NAME, FULCRUM_UP);
            if(fulcrumCycleTime.milliseconds() > 500){
                stateMap.put(constants.PIXEL_CYCLE_FULCRUM, constants.PIXEL_CYCLE_STATE_COMPLETE);
            }
        }
        selectTransition();
    }

    private boolean cycleIsInProgress(){
        String fulcrumCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM);
        if(fulcrumCycleState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            return true;
        }
        return false;
    }
    private void selectTransition(){
        String desiredState = (String) stateMap.get(FULCRUM_SYSTEM_NAME);
        switch(desiredState){
            case FULCRUM_DOWN:{
                fulcrumDown();
                break;
            }
            case FULCRUM_UP:{
                fulcrumUp();
                break;
            }
        }
    }

    private void fulcrumDown(){
        fulcrumServo.setPosition(down);
    }

    private void fulcrumUp(){
        fulcrumServo.setPosition(up);
    }
}
