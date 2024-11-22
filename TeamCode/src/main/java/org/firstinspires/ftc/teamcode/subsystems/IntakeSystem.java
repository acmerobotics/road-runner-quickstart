package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class IntakeSystem extends Mechanism {

    Intake intake;
    SlidesBase intakeSlides;

    private final DcMotorSimple.Direction leftMotorDirection = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction rightMotorDirection = DcMotorSimple.Direction.REVERSE;
    private static final double kP = 0.006;
    private static final double kI = 0.00001;
    private static final double kD = 0.00002;
    private static final double derivativeLowPassGain = 0.15;
    private static final double integralSumMax = 2500;
    private static final double kV = 0.01;
    private static final double kA = 0.0;
    private static final double kStatic = 0.0;
    private static final double kCos = 0.0;
    private static final double kG = 0.0;
    private static final double lowPassGain = 0.15;

    Servo leftSlidePivot;
    Servo rightSlidePivot;

    public static final double RESET_POS = 0;
    public static final double SHORT_POS = 500;
    public static final double MEDIUM_POS = 1000;
    public static final double LONG_POS = 1600;
    private double pivotTargetPosition = PIVOT_UP_POSITION;

    private final static double PIVOT_DOWN_POSITION = 0.05;
    private final static double PIVOT_UP_POSITION = 0.43;


    public enum PivotState {
        PIVOT_DOWN, PIVOT_UP, PIVOT_CUSTOM
    }

    private PivotState activePivotState = PivotState.PIVOT_DOWN;


    enum SlidesPosition {
        RESET, SHORT, MEDIUM, LONG
    }
    public SlidesPosition activeSlidesPosition = SlidesPosition.RESET;

    @Override
    public void init(HardwareMap hwMap) {
        intake = new Intake();
        intake.init(hwMap);

        intakeSlides = new SlidesBase(ConfigurationInfo.leftIntakeSlide.getDeviceName(), ConfigurationInfo.rightIntakeSlide.getDeviceName(),
                leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        intakeSlides.init(hwMap);

        leftSlidePivot = hwMap.get(Servo.class, ConfigurationInfo.leftIntakeSlidePivot.getDeviceName());
        rightSlidePivot = hwMap.get(Servo.class, ConfigurationInfo.rightIntakeSlidePivot.getDeviceName());
        rightSlidePivot.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        switch(activeSlidesPosition) {
            case RESET:
                resetState();
                break;
            case SHORT:
                shortState();
                break;
            case MEDIUM:
                mediumState();
                break;
            case LONG:
                longState();
                break;
        }
        switch (activePivotState) {
            case PIVOT_DOWN:
                pivotDownState();
                break;
            case PIVOT_UP:
                pivotUpState();
                break;
            case PIVOT_CUSTOM:
                break;
        }
        pivotToPosition(pivotTargetPosition);
        intake.loop(aimpad);
        intakeSlides.loop(aimpad, aimpad2);
    }

    public void setSlidesPosition(IntakeSystem.SlidesPosition activeSlidesPosition) {
        intakeSlides.setActiveControlState(SlidesBase.SlidesControlState.AUTONOMOUS);
        this.activeSlidesPosition = activeSlidesPosition;
    }

    /**
     * Set the slides to the reset position
     */
    public void resetState() {
        intakeSlides.setTargetPosition(RESET_POS);
    }

    /**
     * Set the slides to the short position
     */
    public void shortState() {
        intakeSlides.setTargetPosition(SHORT_POS);
    }

    /**
     * Set the slides to the medium position
     */
    public void mediumState() {
        intakeSlides.setTargetPosition(MEDIUM_POS);
    }

    /**
     * Set the slides to the long position
     */
    public void longState() {
        intakeSlides.setTargetPosition(LONG_POS);
    }

    /**
     * Set the state of the slide pivots
     * @param activePivotState the state of the slide pivots (PIVOT_DOWN, PIVOT_UP)
     */
    public void setActivePivotState(PivotState activePivotState) {
        this.activePivotState = activePivotState;
    }

    /**
     * Set the custom position of the slide pivots
     * @param position the custom position of the slide pivots
     */
    public void setPivotStateCustom(double position) {
        activePivotState = PivotState.PIVOT_CUSTOM;
        pivotTargetPosition = position;
    }

    /**
     * Set the slide pivots to the down position
     */
    public void pivotDownState() {
        pivotTargetPosition = PIVOT_DOWN_POSITION;
    }

    /**
     * Set the slide pivots to the up position
     */
    public void pivotUpState() {
        pivotTargetPosition = PIVOT_UP_POSITION;
    }

    /**
     * Set the slide pivots servos to a position clamped by the hinge limits
     */
    public void pivotToPosition(double pivotPosition) {
        double clampedPivotPosition = Math.max(PIVOT_DOWN_POSITION, Math.min(pivotPosition, PIVOT_UP_POSITION));
        leftSlidePivot.setPosition(clampedPivotPosition);
        rightSlidePivot.setPosition(clampedPivotPosition);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        intake.telemetry(telemetry);
    }

    /**
     * Method to individually check each hardware component of the intake system
     * @param aimpad the gamepad used to control the intake system
     */
    public void systemsCheck(AIMPad aimpad, Telemetry telemetry) {
        if (aimpad.isAPressed()) {
            setActivePivotState(PivotState.PIVOT_UP);
        } else if (aimpad.isBPressed()) {
            setActivePivotState(PivotState.PIVOT_DOWN);
        } else if (aimpad.isRightStickMovementEngaged()) {
            setPivotStateCustom(aimpad.getRightStickY());
        } else if (aimpad.isDPadDownPressed()) {
            setSlidesPosition(SlidesPosition.RESET);
        } else if (aimpad.isDPadLeftPressed()) {
            setSlidesPosition(SlidesPosition.SHORT);
        } else if (aimpad.isDPadRightPressed()) {
            setSlidesPosition(SlidesPosition.MEDIUM);
        } else if (aimpad.isDPadUpPressed()) {
            setSlidesPosition(SlidesPosition.LONG);
        } else if (aimpad.isYPressed()) {
            intakeSlides.setActiveControlState(SlidesBase.SlidesControlState.MANUAL);
        }
        intakeSlides.updateManualPower(aimpad.getLeftStickY());
        loop(aimpad);
        telemetry(telemetry);
    }

    private static final SlidesPosition[] SLIDE_POSITIONS = {
            SlidesPosition.RESET,
            SlidesPosition.SHORT,
            SlidesPosition.MEDIUM,
            SlidesPosition.LONG
    };

    public SlidesPosition getNextSlidePosition(SlidesPosition currentPosition) {
        if (currentPosition == SlidesPosition.LONG) {
            return SlidesPosition.LONG;
        }
        int currentIndex = java.util.Arrays.asList(SLIDE_POSITIONS).indexOf(currentPosition);
        int nextIndex = (currentIndex + 1) % SLIDE_POSITIONS.length;
        return SLIDE_POSITIONS[nextIndex];
    }

    public SlidesPosition getPreviousSlidePosition(SlidesPosition currentPosition) {
        if (currentPosition == SlidesPosition.RESET) {
            return SlidesPosition.RESET;
        }
        int currentIndex = java.util.Arrays.asList(SLIDE_POSITIONS).indexOf(currentPosition);
        int previousIndex = (currentIndex - 1 + SLIDE_POSITIONS.length) % SLIDE_POSITIONS.length;
        return SLIDE_POSITIONS[previousIndex];
    }

}
