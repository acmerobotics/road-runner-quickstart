package org.firstinspires.ftc.teamcode.settings;

import com.aimrobotics.aimlib.gamepad.AIMPad;

public class InputHandler {
    public boolean ADVANCE_AUTOMATION = false;
    public boolean TOGGLE_LOW_HANG = false;
    public boolean ADVANCE_HANG = false;
    public boolean BACKWARD_HANG = false;
    public boolean LOW_HEIGHT = false;
    public boolean HIGH_HEIGHT = false;
    public boolean RELEASE_ELEMENT = false;
    public boolean ROTATE_HORIZONTAL = false;
    public boolean RESET_ROTATION = false;
    public boolean TOGGLE_HAND_ARM = false;
    public double SLIDES_CONTROL = 0;
    public boolean MANUAL_OVERRIDE = false;
    public boolean SET_SAMPLE = false;
    public boolean SET_SPECIMEN = false;
    public boolean SET_DUMP = false;

    public void updateInputs(AIMPad aimpad, AIMPad aimpad2) {
        ADVANCE_AUTOMATION = aimpad2.isDPadUpPressed();
        TOGGLE_LOW_HANG = aimpad2.isDPadRightHeld() && aimpad2.isXPressed();
        ADVANCE_HANG = aimpad2.isRightTriggerPressed();
        BACKWARD_HANG = aimpad2.isLeftTriggerPressed();
        LOW_HEIGHT = aimpad2.isLeftBumperPressed();
        HIGH_HEIGHT = aimpad2.isRightBumperPressed();
        RELEASE_ELEMENT = aimpad2.isRightTriggerPressed();
        ROTATE_HORIZONTAL = Math.abs(aimpad2.getRightStickX()) > 0.4;
        RESET_ROTATION = aimpad2.isRightStickPressed();
        TOGGLE_HAND_ARM = aimpad2.isRightTriggerPressed();
        SLIDES_CONTROL = -aimpad2.getLeftStickY();
        MANUAL_OVERRIDE = aimpad2.isDPadLeftHeld();
        SET_SAMPLE = aimpad.isLeftBumperPressed();
        SET_SPECIMEN = aimpad.isRightBumperPressed();
        SET_DUMP = aimpad.isDPadDownPressed();
    }
}
