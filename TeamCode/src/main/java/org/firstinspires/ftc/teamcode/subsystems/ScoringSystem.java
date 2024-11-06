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
                searchingState(aimpad, aimpad2);
                break;
            case TRANSITIONING:
                transitioningState(aimpad, aimpad2);
                break;
            case SLIDES_POSITIONING:
                slidesPositioningState(aimpad, aimpad2);
                break;
        }
        intakeSystem.loop(aimpad);
    }

    public void setActiveScoringState(ScoringState activeScoringState) {
        this.activeScoringState = activeScoringState;

    }
    public void resettingState() {
        intakeSystem.intake.setActiveHingeState(Intake.HingeState.DOWN);
        intakeSystem.setActivePivotState(IntakeSystem.PivotState.PIVOT_DOWN);
        intakeSystem.setAutoSlidesPosition(IntakeSystem.AutoSlidesPosition.RESET);
        if (intakeSystem.intakeSlides.isAtTargetPosition()){
            setActiveScoringState(ScoringState.SEARCHING);
        }
    }
    public void searchingState(AIMPad aimpad, AIMPad aimpad2) {
        intakeSystem.intake.setActiveHingeState(Intake.HingeState.NEUTRAL);
        intakeSystem.setAutoSlidesPosition(IntakeSystem.AutoSlidesPosition.SHORT);

        if (aimpad.isRightBumperPressed()) {
            intakeSystem.setAutoSlidesPosition(intakeSystem.getNextSlidePosition(intakeSystem.activeAutoSlidesPosition));
        } else if (aimpad.isLeftBumperPressed()) {
            intakeSystem.setAutoSlidesPosition(intakeSystem.getPreviousSlidePosition(intakeSystem.activeAutoSlidesPosition));
        }

        if (intakeSystem.intakeSlides.isAtTargetPosition()){
            setActiveScoringState(ScoringState.TRANSITIONING);
        }
    }
    public void transitioningState(AIMPad aimpad, AIMPad aimpad2) {
    }
    public void slidesPositioningState(AIMPad aimpad, AIMPad aimpad2) {
    }
}

