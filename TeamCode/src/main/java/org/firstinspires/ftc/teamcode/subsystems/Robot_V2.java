package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.settings.InputHandler;

public class Robot_V2 extends Mechanism {

    Drivebase drivebase = new Drivebase();
//    Hubs hubs = new Hubs(); // TODO implement hubs
    ScoringAssembly scoringAssembly = new ScoringAssembly();
    Vision vision = new Vision();

    InputHandler inputHandler = new InputHandler();

    enum RobotState {
        RESETTING,
        SEARCHING,
        AUTO_GRASPING,
        RETRACTING,
        PREP_SCORING,
        SCORING
    }

    enum ScoringElement {
        SAMPLE,
        SPECIMEN
    }

    ScoringElement activeScoringElementType = ScoringElement.SAMPLE;

    RobotState activeState = RobotState.RESETTING;

    @Override
    public void init(HardwareMap hwMap) {
        drivebase.init(hwMap);
//        hubs.init(hwMap);
        scoringAssembly.init(hwMap);
        vision.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
//        hubs.loop(aimpad);
        drivebase.loop(aimpad);
        scoringAssembly.loop(aimpad, aimpad2);
        inputHandler.updateInputs(aimpad, aimpad2);

        switch(activeState) {
            case RESETTING:
                resettingState();
                break;
            case SEARCHING:
                searchingState();
                break;
            case AUTO_GRASPING:
                autoGraspingState();
                break;
            case RETRACTING:
                retracting();
                break;
            case PREP_SCORING:
                prepScoringState();
                break;
            case SCORING:
                scoringState();
                break;
        }
    }

    private void resettingState() {
        switch (activeScoringElementType) {
            case SAMPLE:
                scoringAssembly.setPickupResetNeutral();
                break;
            case SPECIMEN:
                scoringAssembly.reset();
        }
        if (scoringAssembly.areMotorsAtTargetPresets()) {
            activeState = RobotState.SEARCHING;
        }
    }

    private void searchingState() {
        switch (activeScoringElementType) {
            case SAMPLE:
                scoringAssembly.slides.setSlidesAtPower(inputHandler.SLIDES_CONTROL);

                if (inputHandler.FLEX_DOWN) {
                    scoringAssembly.multiAxisArm.wrist.flexDown();
                } else if (inputHandler.FLEX_NEUTRAL) {
                    scoringAssembly.multiAxisArm.wrist.flexNeutral();
                }

                if (inputHandler.ROTATE_RIGHT) {
                    scoringAssembly.multiAxisArm.wrist.rotateRight();
                } else if (inputHandler.ROTATE_LEFT) {
                    scoringAssembly.multiAxisArm.wrist.rotateLeft();
                } else {
                    scoringAssembly.multiAxisArm.wrist.rotateCenter();
                }

                if (inputHandler.SWITCH_SCORING_ELEMENT) {

                    activeScoringElementType = ScoringElement.SPECIMEN;
                    activeState = RobotState.RESETTING;
                }
                break;
            case SPECIMEN:
                if (inputHandler.SWITCH_SCORING_ELEMENT) {
                    activeScoringElementType = ScoringElement.SAMPLE;
                    activeState = RobotState.RESETTING;
                }
                break;
        }

        if (inputHandler.TOGGLE_HAND) {
            scoringAssembly.multiAxisArm.hand.toggle();
        }

        if (inputHandler.ADVANCE_AUTOMATION) {
            activeState = RobotState.PREP_SCORING;
        }
    }

    private void autoGraspingState() {
        activeState = RobotState.PREP_SCORING;
    }

    private void retracting() {
        scoringAssembly.setPickupResetClamped();
        if (scoringAssembly.areMotorsAtTargetPresets()) {
            scoringAssembly.setScoringLowBucketClamped();
            activeState = RobotState.PREP_SCORING;
        }
    }

    private void prepScoringState() {
        scoringAssembly.setScoringResetClamped();
        if (scoringAssembly.areMotorsAtTargetPresets()) {
            activeState = RobotState.SCORING;
        }
    }

    private void scoringState() {
        if (inputHandler.HIGH_HEIGHT) {
            scoringAssembly.slides.setSlidesPosition(Slides.SlidesExtension.HIGH_BUCKET);
        } else if (inputHandler.LOW_HEIGHT) {
            scoringAssembly.slides.setSlidesPosition(Slides.SlidesExtension.LOW_BUCKET);
        }

        if (inputHandler.RELEASE_ELEMENT) {
            scoringAssembly.multiAxisArm.hand.open();
            activeState = RobotState.RESETTING;
        }
    }
}
