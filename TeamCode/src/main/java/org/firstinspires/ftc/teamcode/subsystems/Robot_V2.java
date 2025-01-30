package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot_V2 extends Mechanism {

    Drivebase drivebase = new Drivebase();
    Hubs hubs = new Hubs();
    ScoringAssembly scoringAssembly = new ScoringAssembly();
    Vision vision = new Vision();

    enum RobotState {
        RESETTING,
        SEARCHING,
        AUTO_GRASPING,
        RETRACTING,
        PREP_SCORING,
        SCORING
    }

    RobotState activeState = RobotState.RESETTING;

    @Override
    public void init(HardwareMap hwMap) {
        drivebase.init(hwMap);
        hubs.init(hwMap);
        scoringAssembly.init(hwMap);
        vision.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        hubs.loop(aimpad);
        drivebase.loop(aimpad);
        scoringAssembly.loop(aimpad, aimpad2);

        switch(activeState) {
            case RESETTING:
                resettingState(aimpad, aimpad2);
                break;
            case SEARCHING:
                searchingState(aimpad, aimpad2);
                break;
            case AUTO_GRASPING:
                autoGraspingState(aimpad, aimpad2);
                break;
            case RETRACTING:
                retracting(aimpad, aimpad2);
                break;
            case PREP_SCORING:
                prepScoringState(aimpad, aimpad2);
                break;
            case SCORING:
                scoringState(aimpad, aimpad2);
                break;
        }
    }

    private void resettingState(AIMPad aimpad, AIMPad aimpad2) {
        scoringAssembly.reset();
        if (scoringAssembly.areMotorsAtTarget()) {
            scoringAssembly.setPickupResetNeutral();
            activeState = RobotState.SEARCHING;
        }
    }

    private void searchingState(AIMPad aimpad, AIMPad aimpad2) {
        if (scoringAssembly.pivot.isAtTargetPosition()) {
            scoringAssembly.slides.setSlidesAtPower(-aimpad2.getLeftStickY());

            if (aimpad2.isRightTriggerReleased()) {
                scoringAssembly.multiAxisArm.hand.toggle();
            }

            if (aimpad2.isAReleased()) {
                scoringAssembly.multiAxisArm.wrist.flexDown();
            } else if (aimpad2.isYReleased()) {
                scoringAssembly.multiAxisArm.wrist.flexNeutral();
            }

            if (aimpad2.getRightStickX() > 0.3) {
                scoringAssembly.multiAxisArm.wrist.rotateRight();
            } else if (aimpad2.getRightStickX() < -0.3) {
                scoringAssembly.multiAxisArm.wrist.rotateLeft();
            } else {
                scoringAssembly.multiAxisArm.wrist.rotateCenter();
            }

            if (aimpad2.isDPadUpPressed()) {
                activeState = RobotState.PREP_SCORING;
            }
        }
    }

    private void autoGraspingState(AIMPad aimpad, AIMPad aimpad2) {
        activeState = RobotState.PREP_SCORING;
    }

    private void retracting(AIMPad aimpad, AIMPad aimpad2) {
        scoringAssembly.setPickupResetClamped();
        if (scoringAssembly.areMotorsAtTarget()) {
            scoringAssembly.setScoringLowBucketClamped();
            activeState = RobotState.PREP_SCORING;
        }
    }

    private void prepScoringState(AIMPad aimpad, AIMPad aimpad2) {
        scoringAssembly.setScoringResetClamped();
        if (scoringAssembly.areMotorsAtTarget()) {

            activeState = RobotState.SCORING;
        }
    }

    private void scoringState(AIMPad aimpad, AIMPad aimpad2) {
        activeState = RobotState.RESETTING;
    }
}
