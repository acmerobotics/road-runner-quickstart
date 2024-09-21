package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.New.SubSystems.JavaSubsystems;

public class LimitSwitch implements JavaSubsystems {

    DigitalChannel digitalChannel;
    State state;
    public static int resetValue;


    public LimitSwitch(DigitalChannel digitalChannel, int resetValue) {
        this.digitalChannel = digitalChannel;
        LimitSwitch.resetValue = resetValue;
    }
    @Override
    public void update() {
        boolean limitSwitchState = digitalChannel.getState();

        if(limitSwitchState){
            state = State.RESET;
        }
        else{
            state = State.DONT;
        }

    }
    enum State{
        RESET(resetValue),
        DONT(0);
        final int value;
        State(int value) {
            this.value = value;
        }
    }

}
