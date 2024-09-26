package org.firstinspires.ftc.teamcode.New.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HangPracticeJohn implements JavaSubsystems{
    HardwareMap hardwareMap;
    Servo hangRelease;
    public HangState hangstate;
    

    //hypotheical hang positions
    double closed = 0;
    double released = 0.25;

    public HangPracticeJohn(HardwareMap hardwareMap){
        hangRelease = hardwareMap.get(Servo.class, "Hang");
    }
    public void update(){
        if (hangstate == HangState.CLOSED){
                hangRelease.setPosition(closed);
        }
        if (hangstate == HangState.RELEASED){
            hangRelease.setPosition(released);
        }
    }
    public enum HangState {
        RELEASED,
        CLOSED
    }

}
