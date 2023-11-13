package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Fulcrum {
    private Telemetry telemetry;
    private Map stateMap;
    private ServoImplEx fulcrumServo;
    private final double down = 0.0;
    private final double up = 1.0;
    private double lowerPWMLimit;
    private double upperPWMLimit;

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

    public void set_state(){
        selectTransition();
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
