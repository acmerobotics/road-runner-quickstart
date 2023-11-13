package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;
public class Virtual4Bar {
    private Telemetry telemetry;
    private ServoImplEx virtual4Bar;
    private Map stateMap;
    private double VIRTUAL_4BAR_RETRACTED_POSITION;
    private double VIRTUAL_4BAR_EXTENDED_POSITION;
    //Statemap strings
    public final String VIRTUAL_4BAR_SYSTEM_NAME = "VIRTUAL_4BAR_SYSTEM_NAME";
    public final String VIRTUAL_4BAR_RETRACTED_STATE = "VIRTUAL_4BAR_RETRACTED_STATE";
    public final String VIRTUAL_4BAR_EXTENDED_STATE = "VIRTUAL_4BAR_EXTENDED_STATE";

    //Make these 2 variables constants once figured out
    private double low_PWM_cap;
    private double high_PWM_cap;
    public Virtual4Bar(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry =  telemetry;
        this.stateMap = stateMap;

        virtual4Bar = (ServoImplEx) hwMap.servo.get("Virtual4Bar");
        virtual4Bar.setPwmRange(new PwmControl.PwmRange(low_PWM_cap, high_PWM_cap));

    }

    public void setState(String desiredState){
        selectTransition(desiredState);
    }

    private void selectTransition(String desiredState){
        switch(desiredState){
            case VIRTUAL_4BAR_RETRACTED_STATE:{
                retract();
                break;
            }
            case VIRTUAL_4BAR_EXTENDED_STATE:{
                extend();
                break;
            }
        }
    }

    private void retract(){
        virtual4Bar.setPosition(VIRTUAL_4BAR_RETRACTED_POSITION);
    }

    private void extend(){
        virtual4Bar.setPosition(VIRTUAL_4BAR_EXTENDED_POSITION);
    }
}
