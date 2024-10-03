package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.New.SubSystems.JavaSubsystems;

public class LimitSwitch implements JavaSubsystems {
    public State state;
    DigitalChannel digitalChannel;
    public int resetValue;

    public LimitSwitch(DigitalChannel digitalChannel, int resetValue) {
        this.digitalChannel = digitalChannel;
        this.resetValue = resetValue;
        state = State.RELEASED;
    }
    public enum State{
        PRESSED,
        RELEASED
    }

    @Override
    public void update() {

        if(digitalChannel.getState()){
            state = State.PRESSED;
        }
        else{
            state = State.RELEASED;
        }
    }

}
