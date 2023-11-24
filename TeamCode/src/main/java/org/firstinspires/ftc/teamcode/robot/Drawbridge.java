package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Drawbridge {
    public final String DRAWBRIDGE_SYSTEM_NAME = "DRAWBRIDGE_SYSTEM_NAME";
    public final String DRAWBRIDGE_UP_STATE = "DRAWBRIDGE_UP_STATE";

    public final String DRAWBRIDGE_DOWN_STATE = "DRAWBRIDGE_DOWN_STATE";




    private int drawbridgePWMLowerLimit = 850;
    private int drawbridgePWMHigherLimit = 1600;

    private int hardstopPWMLowerLimit;
    private int hardstopPWMHigherLimit;
    private Telemetry telemetry;

    private Map stateMap;

    private ServoImplEx drawBridge;

    private ServoImplEx hardStop;

    private double drawBridgeUpPosition = 1;

    private double drawBridgeDownPosition = 0;

    public Drawbridge(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        drawBridge = hwMap.get(ServoImplEx.class, "Drawbridge");
        hardStop = hwMap.get(ServoImplEx.class, "intakeHardstop");

        drawBridge.setPwmRange(new PwmControl.PwmRange(drawbridgePWMLowerLimit, drawbridgePWMHigherLimit));
        hardStop.setPwmRange(new PwmControl.PwmRange(hardstopPWMLowerLimit, hardstopPWMHigherLimit));

    }

    public void setState(){
        selectTransition();
    }

    private void selectTransition(){
        String state = (String) stateMap.get(DRAWBRIDGE_SYSTEM_NAME);
        telemetry.addData("State of drawbridge", state);
        switch(state){
            case DRAWBRIDGE_UP_STATE:{
                setDrawBridgeUp();
                break;
            }
            case DRAWBRIDGE_DOWN_STATE:{
                setDrawBridgeDown();
                break;
            }
        }
    }

    public void setDrawBridgeUp(){
        telemetry.addData("Drawbridge State", "up");
        drawBridge.setPosition(1);
    }
    public void setDrawBridgeDown(){
        telemetry.addData("Drawbridge State", "down");
        drawBridge.setPosition(0);
    }
}
