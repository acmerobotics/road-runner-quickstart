package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.InputHandler;

public class Robot_V2 extends Mechanism {

    boolean isAuto;

    public Drivebase drivebase;
//    Hubs hubs = new Hubs(); // TODO implement hubs
    public ScoringAssembly scoringAssembly = new ScoringAssembly();
//    Vision vision = new Vision();

    InputHandler inputHandler = new InputHandler();

    enum RobotState {
        RESETTING,
        SEARCHING,
        AUTO_GRASPING,
        RETRACTING,
        PREP_SCORING,
        SCORING,
        DROP_SLIDES,
        LOW_HANG,
        TOTAL_FIX
    }

    enum ScoringElement {
        SAMPLE,
        SPECIMEN
    }

    ScoringElement activeScoringElementType = ScoringElement.SPECIMEN;

    RobotState activeState = RobotState.RESETTING;

    Pose2d startingPose;

    public Robot_V2(Pose2d startingPose, boolean isAuto) {
        this.startingPose = startingPose;
        this.isAuto = isAuto;
        drivebase = new Drivebase(startingPose);
    }

    @Override
    public void init(HardwareMap hwMap) {
        drivebase.init(hwMap);
//        hubs.init(hwMap);
        scoringAssembly.init(hwMap);
//        vision.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
//        hubs.loop(aimpad);
        scoringAssembly.loop(aimpad, aimpad2);
        if (!isAuto) {
            inputHandler.updateInputs(aimpad, aimpad2);
            drivebase.loop(aimpad);
            switch (activeState) {
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
                case DROP_SLIDES:
                    dropSlides();
                    break;
                case LOW_HANG:
                    lowHang();
                    break;
                case TOTAL_FIX:
                    totalFix();
            }
        }

        if (inputHandler.MANUAL_OVERRIDE) {
            activeState = RobotState.TOTAL_FIX;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        scoringAssembly.telemetry(telemetry);
        telemetry.addData("Current State:", activeState);
        telemetry.addData("Current Element:", activeScoringElementType);
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
            if (activeScoringElementType == ScoringElement.SPECIMEN) {
                scoringAssembly.resetSpecimen();
            }
            activeState = RobotState.SEARCHING;
        }
    }

    private void searchingState() {
        switch (activeScoringElementType) {
            case SAMPLE:
                scoringAssembly.slides.setSlidesAtPower(inputHandler.SLIDES_CONTROL);

                if (inputHandler.FLEX_DOWN) {
                    scoringAssembly.multiAxisArm.wrist.flexDown();
                    scoringAssembly.multiAxisArm.elbow.middle();
                } else if (inputHandler.FLEX_NEUTRAL) {
                    scoringAssembly.multiAxisArm.wrist.flexNeutral();
                } else if (inputHandler.FLEX_DOWN_PLUS) {
                    scoringAssembly.multiAxisArm.wrist.flexDown();
                    scoringAssembly.multiAxisArm.elbow.pickupPlus();
                }

                if (inputHandler.ROTATE_RIGHT) {
                    scoringAssembly.multiAxisArm.wrist.rotateRight();
                } else if (inputHandler.ROTATE_LEFT) {
                    scoringAssembly.multiAxisArm.wrist.rotateLeft();
                } else if (inputHandler.RESET_ROTATION) {
                    scoringAssembly.multiAxisArm.wrist.rotateCenter();
                }

                if (inputHandler.SPECIMEN_ADVANCE) {
                    activeScoringElementType = ScoringElement.SPECIMEN;
                    activeState = RobotState.RESETTING;
                }

                if (inputHandler.SAMPLE_ADVANCE) {
                    activeState = RobotState.PREP_SCORING;
                }
                break;
            case SPECIMEN:
                if (inputHandler.SAMPLE_ADVANCE) {
                    activeScoringElementType = ScoringElement.SAMPLE;
                    activeState = RobotState.RESETTING;
                }

                if (inputHandler.SPECIMEN_ADVANCE) {
                    activeState = RobotState.PREP_SCORING;
                }
                break;
        }

        if (inputHandler.TOGGLE_HAND_ARM) {
            scoringAssembly.multiAxisArm.hand.toggle();
        }

        if (inputHandler.TOGGLE_LOW_HANG) {
            scoringAssembly.setLowHangRetracted();
            activeState = RobotState.LOW_HANG;
        }
    }

    private void autoGraspingState() {
        activeState = RobotState.PREP_SCORING;
    }

    private void retracting() {
        scoringAssembly.setPickupResetClamped();
        if (scoringAssembly.areMotorsAtTargetPresets()) {
            activeState = RobotState.PREP_SCORING;
        }
    }

    private void prepScoringState() {
        switch (activeScoringElementType) {
            case SPECIMEN:
                scoringAssembly.setSpecimenClamped();
                break;
            case SAMPLE:
                scoringAssembly.setScoringResetClamped();
                break;
        }
        if (scoringAssembly.areMotorsAtTargetPresets()) {
            activeState = RobotState.SCORING;
        }
        if (inputHandler.BACK_TO_SEARCH) {
            scoringAssembly.setPickupResetNeutralClosed();
            activeState = RobotState.SEARCHING;
        }
    }

    private void scoringState() {
        switch (activeScoringElementType) {
            case SPECIMEN:
                if (inputHandler.TOGGLE_HAND_ARM) {
                    scoringAssembly.multiAxisArm.toggleSpecimen();
                }
                break;
            case SAMPLE:
                if (inputHandler.HIGH_HEIGHT) {
                    scoringAssembly.slides.setSlidesPosition(Slides.SlidesExtension.HIGH_BUCKET);
                } else if (inputHandler.LOW_HEIGHT) {
                    scoringAssembly.slides.setSlidesPosition(Slides.SlidesExtension.LOW_BUCKET);
                }

                if (inputHandler.RELEASE_ELEMENT) {
                    scoringAssembly.multiAxisArm.hand.open();
                    scoringAssembly.multiAxisArm.wrist.flexNeutral();
                }
                break;
        }
        if (inputHandler.SAMPLE_ADVANCE || inputHandler.SPECIMEN_ADVANCE) {
            scoringAssembly.reset();
            activeState = RobotState.DROP_SLIDES;
        }
    }

    private void dropSlides() {
        if (scoringAssembly.areMotorsAtTarget()) {
           activeState =  RobotState.RESETTING;
        }
    }

    private void lowHang() {
        if (inputHandler.EXTEND_SLIDES) {
            scoringAssembly.setLowHangExtended();
        } else if (inputHandler.RETRACT_SLIDES) {
            scoringAssembly.setLowHangRetracted();
        } else if (inputHandler.TOGGLE_LOW_HANG) {
            activeState = RobotState.RESETTING;
        }
    }

    private void totalFix() {
        scoringAssembly.totalFix();
        if (inputHandler.ADVANCE_AUTOMATION) {
            scoringAssembly.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scoringAssembly.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scoringAssembly.slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            scoringAssembly.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            activeState = RobotState.RESETTING;
        }
    }
}
