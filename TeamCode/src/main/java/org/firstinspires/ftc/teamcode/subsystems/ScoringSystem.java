package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringSystem extends Mechanism {
    IntakeSystem intakeSystem;
    OuttakeSystem outtakeSystem;

    enum ScoringState {
        RESETTING, SEARCHING, TRANSITIONING, SLIDES_POSITIONING
    }

    @Override
    public void init(HardwareMap hwMap) {
        intakeSystem = new IntakeSystem();
        intakeSystem.init(hwMap);

        outtakeSystem = new OuttakeSystem();
        outtakeSystem.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {

    }
}
