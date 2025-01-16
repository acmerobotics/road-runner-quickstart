package org.firstinspires.ftc.teamcode.subsystems.v1;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.subsystems.generic.SlidesBase;

public class ScoringSystem extends Mechanism {

    public IntakeSystem intakeSystem = new IntakeSystem();
    public OuttakeSystem outtakeSystem = new OuttakeSystem();
    public SpecimenGrabber specimenGrabber = new SpecimenGrabber();

    public enum ScoringState {
        RESETTING, SEARCHING, TRANSITIONING1, TRANSITIONING2, SLIDES_POSITIONING, SPECIMEN_POSITIONING, SPECIMEN_SCORE, OUTTAKE_REZEROING, INTAKE_REZEROING
    }

    ScoringState activeScoringState = ScoringState.RESETTING;

    ScoringState outtakeRezeroingGoBackState = ScoringState.SEARCHING;


    @Override
    public void init(HardwareMap hwMap) {
        intakeSystem.init(hwMap);
        outtakeSystem.init(hwMap);
        specimenGrabber.init(hwMap);
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
            case TRANSITIONING1:
                transitioning1State(aimpad, aimpad2);
                break;
            case TRANSITIONING2:
                transitioning2State(aimpad, aimpad2);
                break;
            case SLIDES_POSITIONING:
                slidesPositioningState(aimpad, aimpad2);
                break;
            case SPECIMEN_POSITIONING:
                specimenPositioningState(aimpad, aimpad2);
                break;
            case SPECIMEN_SCORE:
                specimenScoreState(aimpad, aimpad2);
                break;
            case OUTTAKE_REZEROING:
                outtakeRezeroing();
                break;
            case INTAKE_REZEROING:
                intakeRezeroing();
                break;
        }
        intakeSystem.loop(aimpad, aimpad2);
        outtakeSystem.loop(aimpad, aimpad2);
        specimenGrabber.loop(aimpad);
    }

    public void setActiveScoringState(ScoringState activeScoringState) {
        this.activeScoringState = activeScoringState;
    }

    public void resettingState() {
        intakeSystem.multiAxisArm.resetClosed();

        intakeSystem.pivotDown();

        outtakeSystem.outtake.armIn();
        outtakeSystem.outtake.bucketIn();

        specimenGrabber.setGrabberState(SpecimenGrabber.GrabberState.RELEASE);

        intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.RESET);
        outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.RESET);
        if (intakeSystem.intakeSlides.isAtTargetPosition() && outtakeSystem.outtakeSlides.isAtTargetPosition()){
            intakeSystem.multiAxisArm.searchingNeutral();
            setActiveScoringState(ScoringState.SEARCHING);
        }
    }
    public void searchingState(AIMPad aimpad, AIMPad aimpad2) {
        specimenGrabber.setGrabberState(SpecimenGrabber.GrabberState.RELEASE);


        // IntakeSlides control
        if (aimpad2.isRightBumperPressed()) {
            intakeSystem.setSlidesPosition(intakeSystem.getNextSlidePosition(intakeSystem.activeSlidesPosition));
        } else if (aimpad2.isLeftBumperPressed()) {
            intakeSystem.setSlidesPosition(intakeSystem.getPreviousSlidePosition(intakeSystem.activeSlidesPosition));
        }

        if (aimpad2.isAPressed()) {
            intakeSystem.multiAxisArm.searchingDownOpen();
        } else if (aimpad2.isBPressed()) {
            intakeSystem.multiAxisArm.searchingNeutral();
        } else if (aimpad2.isYPressed()) {
            intakeSystem.multiAxisArm.searchingDownClosed();
        }

        if (aimpad2.isDPadUpPressed()) {
            setActiveScoringState(ScoringState.TRANSITIONING1);
        }

        if (aimpad2.isDPadDownPressed()) {
            specimenGrabber.setGrabberState(SpecimenGrabber.GrabberState.GRAB);
            setActiveScoringState(ScoringState.SPECIMEN_POSITIONING);
        }

        // SlIDE REZEROING
        if (aimpad.isDPadUpPressed()) {
            outtakeRezeroingGoBackState = ScoringState.SEARCHING;
            setActiveScoringState(ScoringState.OUTTAKE_REZEROING);
        }

        if (aimpad.isDPadDownPressed()) {
            intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.RESET);
            setActiveScoringState(ScoringState.INTAKE_REZEROING);
        }
    }

    public void transitioning1State(AIMPad aimpad, AIMPad aimpad2) {
        intakeSystem.multiAxisArm.resetClosed();
        intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.RESET);

        if (aimpad.isDPadUpPressed()) {
            outtakeRezeroingGoBackState = ScoringState.TRANSITIONING1;
            setActiveScoringState(ScoringState.OUTTAKE_REZEROING);
        }

        if (intakeSystem.intakeSlides.isAtTargetPosition()) {
            if (aimpad2.isDPadUpPressed()) {
                setActiveScoringState(ScoringState.TRANSITIONING2);
            } else if (aimpad2.isYPressed()) {
                setActiveScoringState(ScoringState.SEARCHING);
            }
        }
    }

    public void transitioning2State(AIMPad aimpad, AIMPad aimpad2) {
        intakeSystem.multiAxisArm.resetOpen();
        if (aimpad2.isDPadUpPressed()) {
            setActiveScoringState(ScoringState.SLIDES_POSITIONING);
        }
    }

    public void slidesPositioningState(AIMPad aimpad, AIMPad aimpad2) {
        intakeSystem.multiAxisArm.resetAvoid();
        outtakeSystem.outtake.armOut();
        if (aimpad2.getRightTrigger() > GamepadSettings.GP1_TRIGGER_DEADZONE) {
            outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.TALL);
        } else if (aimpad2.getLeftTrigger() > GamepadSettings.GP1_TRIGGER_DEADZONE) {
            outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SHORT);
        }

        if (aimpad2.isLeftBumperPressed() && aimpad2.isRightBumperPressed()) {
            outtakeSystem.outtake.bucketOut();
        }

        if (aimpad2.isDPadUpPressed()) {
            setActiveScoringState(ScoringState.RESETTING);
        }
    }

    public void specimenPositioningState(AIMPad aimpad, AIMPad aimpad2) {
        intakeSystem.setSlidesPosition(IntakeSystem.SlidesPosition.RESET);
        intakeSystem.multiAxisArm.resetAvoid();


        if (aimpad.isDPadUpPressed()) {
            outtakeRezeroingGoBackState = ScoringState.SPECIMEN_POSITIONING;
            setActiveScoringState(ScoringState.OUTTAKE_REZEROING);
        }

        if (aimpad2.isBPressed()) {
            setActiveScoringState(ScoringState.SEARCHING);
        }

        if (aimpad2.getRightTrigger() > GamepadSettings.GP1_TRIGGER_DEADZONE) {
            outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_HIGH);
        } else if (aimpad2.getLeftTrigger() > GamepadSettings.GP1_TRIGGER_DEADZONE) {
            outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_LOW);
        }

        if (aimpad2.isDPadUpPressed()) {
            if (outtakeSystem.activeSlidesPosition == OuttakeSystem.SlidesPosition.SPECIMEN_HIGH) {
                outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_HIGH_DROP);
            } else {
                outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_LOW_DROP);
            }
            setActiveScoringState(ScoringState.SPECIMEN_SCORE);
        }
    }

    public void specimenScoreState(AIMPad aimpad, AIMPad aimpad2) {

        if (aimpad2.getLeftTrigger() > GamepadSettings.GP1_TRIGGER_DEADZONE) {
            outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_LOW);
            setActiveScoringState(ScoringState.SPECIMEN_POSITIONING);
        } else if (aimpad2.getRightTrigger() > GamepadSettings.GP1_TRIGGER_DEADZONE) {
            outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.SPECIMEN_HIGH);
            setActiveScoringState(ScoringState.SPECIMEN_POSITIONING);
        }
        if (outtakeSystem.outtakeSlides.isAtTargetPosition() && aimpad2.isDPadUpPressed()) {
            specimenGrabber.setGrabberState(SpecimenGrabber.GrabberState.RELEASE);
            setActiveScoringState(ScoringState.RESETTING);
        }

    }

    public void outtakeRezeroing() {
        outtakeSystem.outtakeSlides.updateManualPower(-0.3);
        outtakeSystem.outtakeSlides.setActiveControlState(SlidesBase.SlidesControlState.MANUAL);
        if (outtakeSystem.outtakeSlides.currentSpikeDetected()) {
            outtakeSystem.outtakeSlides.updateManualPower(0);
            outtakeSystem.outtakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSystem.outtakeSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSystem.setSlidesPosition(OuttakeSystem.SlidesPosition.RESET);
            setActiveScoringState(outtakeRezeroingGoBackState);
        }
    }

    public void intakeRezeroing() {
        intakeSystem.intakeSlides.updateManualPower(-0.3);
        intakeSystem.intakeSlides.setActiveControlState(SlidesBase.SlidesControlState.MANUAL);
        if (intakeSystem.intakeSlides.currentSpikeDetected()) {
            intakeSystem.intakeSlides.updateManualPower(0);
            intakeSystem.intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSystem.intakeSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setActiveScoringState(ScoringState.TRANSITIONING1);
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Automation State", activeScoringState);
    }
}

