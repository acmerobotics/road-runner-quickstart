package org.firstinspires.ftc.teamcode.settings;

import com.aimrobotics.aimlib.gamepad.AIMPad;

public class InputHandler {
    public boolean ADVANCE_AUTOMATION = false;
    public boolean SWITCH_SCORING_ELEMENT = false;
    public boolean LOW_HEIGHT = false;
    public boolean HIGH_HEIGHT = false;
    public boolean UP_A_LITTLE = false;
    public boolean DOWN_A_LITTLE = false;
    public boolean RELEASE_ELEMENT = false;
    public boolean ROTATE_LEFT = false;
    public boolean ROTATE_RIGHT = false;
    public boolean FLEX_DOWN = false;
    public boolean FLEX_NEUTRAL = false;
    public boolean TOGGLE_HAND = false;
    public double SLIDES_CONTROL = 0;

    public void updateInputs(AIMPad aimpad, AIMPad aimpad2) {
        ADVANCE_AUTOMATION = aimpad2.isDPadUpReleased();
        SWITCH_SCORING_ELEMENT = aimpad2.isDPadDownReleased();
        LOW_HEIGHT = aimpad2.isLeftBumperReleased();
        HIGH_HEIGHT = aimpad2.isRightBumperReleased();
        RELEASE_ELEMENT = aimpad2.isRightTriggerReleased();
        ROTATE_LEFT = aimpad2.getRightStickX() < -0.3;
        ROTATE_RIGHT = aimpad2.getRightStickX() > 0.3;
        FLEX_DOWN = aimpad2.isAReleased();
        FLEX_NEUTRAL = aimpad2.isYReleased();
        TOGGLE_HAND = aimpad2.isRightTriggerReleased();
        SLIDES_CONTROL = -aimpad2.getLeftStickY();
    }
}
