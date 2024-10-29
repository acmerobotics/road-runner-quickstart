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

    ScoringState activeScoringState = ScoringState.RESETTING;


    @Override
    public void init(HardwareMap hwMap) {
        intakeSystem = new IntakeSystem();
        intakeSystem.init(hwMap);

        outtakeSystem = new OuttakeSystem();
        outtakeSystem.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        switch(activeScoringState) {
            case RESETTING:
                resettingState();
                break;
            case SEARCHING:
                searchingState();
                break;
            case TRANSITIONING:
                transitioningState();
                break;
            case SLIDES_POSITIONING:
                slidesPositioningState();
                break;
        }
    }

    public void setActiveScoringState(ScoringState activeScoringState) {
        this.activeScoringState = activeScoringState;

    }
    public void resettingState() {
        intakeSystem.intake.setActiveHingeState(Intake.HingeState.DOWN);
        intakeSystem.setActivePivotState(IntakeSystem.PivotState.DOWN);
        intakeSystem.intakeSlides.setTargetPosition(intakeSystem.SLIDES_RESET_POS);
        if (!intakeSystem.intakeSlides.isAtTargetPosition()){
            intakeSystem.intakeSlides.update();
        } else {
            setActiveScoringState(ScoringState.SEARCHING);

        }
    }
    public void searchingState(AIMPad aimpad, AIMPad aimpad2) {
        intakeSystem.intakeSlides.setTargetPosition(aimpad.getLeftStickX());
        intakeSystem.intake.hingeToPosition(aimpad.getRightStickX());

    }
    public void transitioningState(AIMPad aimpad, AIMPad aimpad2) {
    }
    public void slidesPositioningState(AIMPad aimpad, AIMPad aimpad2) {
    }
}

