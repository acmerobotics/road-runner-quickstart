package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.New.SubSystems.JavaSubsystems;

public class LimitSwitch implements JavaSubsystems {

    DigitalChannel digitalChannel;
    State state;
    public int resetValue;

    public LimitSwitch(DigitalChannel digitalChannel, int resetValue) {
        this.digitalChannel = digitalChannel;
        this.resetValue = resetValue;
    }

    @Override
    public void update() {
        boolean limitSwitchState = digitalChannel.getState();

        if(limitSwitchState){
            state = State.PRESSED;
        }
        else{
            state = State.RELEASED;
        }
    }
    enum State{
        PRESSED,
        RELEASED
    }
}
