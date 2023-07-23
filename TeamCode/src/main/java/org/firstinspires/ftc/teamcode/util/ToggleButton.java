package org.firstinspires.ftc.teamcode.util;

public class ToggleButton {
    private boolean previousState = false, currentState = false;
    private boolean toggle = false;

    public ToggleButton() {
        toggle = false;
    }

    public ToggleButton(boolean initalToggleState)
    {
        toggle = initalToggleState;
    }

    public boolean getState()
    {
        return toggle;
    }

    public void setState(boolean state) {
        toggle = state;
        previousState = false;
        currentState = false;
    }

    public boolean update(boolean currentState) {
        previousState = this.currentState;
        this.currentState = currentState;

        if (currentState && currentState != previousState)
            toggle = !toggle;

        return toggle;
    }
}