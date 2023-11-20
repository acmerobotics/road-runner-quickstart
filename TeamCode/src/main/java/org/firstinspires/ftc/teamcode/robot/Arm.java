package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Arm {
    public final String ARM_SYSTEM_NAME = "ARM_SYSTEM_NAME";
    public final String ARM_DEPOSIT_STATE = "ARM_DEPOSIT_STATE";
    public final String ARM_IDLE_STATE = "ARM_IDLE_STATE";
    public ServoImplEx rightArmServo;
    public ServoImplEx leftArmServo;
    private double arm_deposit_position = 1.0;
    private double arm_idle_position = 0.0;
    private Telemetry telemetry;
    private Map stateMap;

    public AnalogSensor axonEncoder;

    private double lowerPWMLimit;
    private double upperPWMLimit;

    public Arm(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        rightArmServo = hwMap.get(ServoImplEx.class, "rightArmServo");
        leftArmServo = hwMap.get(ServoImplEx.class, "leftArmServo");

        rightArmServo.setPwmRange(new PwmControl.PwmRange(lowerPWMLimit,upperPWMLimit));
        leftArmServo.setPwmRange(new PwmControl.PwmRange(lowerPWMLimit,upperPWMLimit));

    }

    public void setState(){
        selectTransition();
    }

    private void selectTransition(){
        String desiredState = (String) stateMap.get(ARM_SYSTEM_NAME);
        switch (desiredState){
            case ARM_DEPOSIT_STATE:{
                armToDepositPosition();
            }

            case ARM_IDLE_STATE:{
                armToIdlePosition();
            }
        }

    }

    private void armToDepositPosition(){
        leftArmServo.setPosition(arm_deposit_position);
        rightArmServo.setPosition(arm_deposit_position);
    }
    private void armToIdlePosition(){
        leftArmServo.setPosition(arm_idle_position);
        rightArmServo.setPosition(arm_idle_position);
    }
}
