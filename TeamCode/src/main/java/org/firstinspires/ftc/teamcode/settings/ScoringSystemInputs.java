package org.firstinspires.ftc.teamcode.settings;

import com.aimrobotics.aimlib.gamepad.AIMPad;

public class ScoringSystemInputs {

    public boolean ADVANCE_AUTOMATION;

    public ScoringSystemInputs(AIMPad aimPad, AIMPad aimPad2) {
        updateInputs(aimPad, aimPad2);
    }

    public void updateInputs(AIMPad aimPad, AIMPad aimPad2) {
        ADVANCE_AUTOMATION = aimPad.isAPressed();
    }
}
