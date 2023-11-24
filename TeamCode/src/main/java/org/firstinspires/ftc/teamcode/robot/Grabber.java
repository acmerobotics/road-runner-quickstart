package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Grabber {
    public final String GRABBER_SYSTEM_NAME = "GRABBER_SYSTEM_NAME";
    public final String GRABBER_OPEN_STATE = "GRABBER_OPEN_STATE";
    public final String GRABBER_PICK_1_PIXEL = "GRABBER_PICK_1_PIXEL";
    public final String GRABBER_PICK_2_PIXEL = "GRABBER_PICK_2_PIXEL";
    public final String GRABBER_DEPOSIT_1_PIXEL = "GRABBER_DEPOSIT_1_PIXEL";
    public final String GRABBER_DEPOSIT_2_PIXEL = "GRABBER_DEPOSIT_2_PIXEL";

    private double grabberOpenPosition;
    private double grabberPick1PixelPosition;
    private double grabberPick2PixelPosition;
    private double grabberDeposit1Pixel;
    private double grabberDeposit2Pixel;

    private Telemetry telemetry;
    private Map stateMap;

    private ServoImplEx grabber;

    private int grabberPWMHigherlimit;
    private int grabberPWMLowerLimit;

    public Grabber(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        grabber = hwMap.get(ServoImplEx.class, "grabber");
        grabber.setPwmRange(new PwmControl.PwmRange(grabberPWMLowerLimit, grabberPWMHigherlimit));
    }

    public void setState(){
        selectTransition();
    }

    private void selectTransition(){
        String desiredState = (String) stateMap.get(GRABBER_SYSTEM_NAME);
        switch (desiredState){
            case GRABBER_OPEN_STATE:{
                setPosition(grabberOpenPosition);
            }
            case GRABBER_PICK_1_PIXEL:{
                setPosition(grabberPick1PixelPosition);
            }
            case GRABBER_PICK_2_PIXEL:{
                setPosition(grabberPick2PixelPosition);
            }
        }
    }

    private void setPosition(double position){

    }
}
