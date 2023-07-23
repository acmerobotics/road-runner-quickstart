package org.firstinspires.ftc.teamcode.util;

public class StickyButton {
    private boolean previousState = false, currentState = false;

    public boolean getState()
    {
        //Returns true if the current state of the button is pressed,
        //and it was not pressed before
        return currentState && currentState != previousState;
    }

    public boolean getOppositeState() {
        //Returns true if the current state of the button is not pressed,
        //and it was pressed before
        return !currentState && currentState != previousState;
    }

    public void update(boolean currentState)
    {
        previousState = this.currentState;
        this.currentState = currentState;
    }
}