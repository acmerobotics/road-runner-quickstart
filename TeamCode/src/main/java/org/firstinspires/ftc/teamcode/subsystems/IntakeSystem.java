package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.settings.GamepadSettings;

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


    enum AutoSlidesPosition {
        RESET, SHORT, MEDIUM, LONG
    }
    public AutoSlidesPosition activeAutoSlidesPosition = AutoSlidesPosition.RESET;

    public enum SlidesControlState {
        AUTONOMOUS, MANUAL
    }
    private SlidesControlState activeControlState = SlidesControlState.AUTONOMOUS;

    public double manualPower = 0;

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
    public void loop(AIMPad aimpad) {
        pivotToPosition(pivotTargetPosition);
        intake.loop(aimpad);
        switch (activeControlState){
            case AUTONOMOUS:
                switch(activeAutoSlidesPosition) {
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
                intakeSlides.update();
                break;
            case MANUAL:
                if (Math.abs(manualPower) > GamepadSettings.GP1_STICK_DEADZONE && !intakeSlides.currentSpikeDetected()) {
                    intakeSlides.setPower(manualPower);
                } else {
                    intakeSlides.holdPosition();
                }
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
    }

    public void setManualPower(double power) {
        manualPower = power;
    }
  
    /**
     * Set the control state of the slides
     * @param activeControlState control state (AUTONOUMOUS or MANUAL)
     */
    public void setActiveControlState(SlidesControlState activeControlState) {
        this.activeControlState = activeControlState;
    }

    /**
     * Automatically set the slides to a target position
     * @param activeAutoSlidesPosition slides target state (RESET, SHORT, MEDIUM, LONG)
     */
    public void setAutoSlidesPosition(AutoSlidesPosition activeAutoSlidesPosition) {
        setActiveControlState(SlidesControlState.AUTONOMOUS);
        this.activeAutoSlidesPosition = activeAutoSlidesPosition;
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
            setAutoSlidesPosition(AutoSlidesPosition.RESET);
        } else if (aimpad.isDPadLeftPressed()) {
            setAutoSlidesPosition(AutoSlidesPosition.SHORT);
        } else if (aimpad.isDPadRightPressed()) {
            setAutoSlidesPosition(AutoSlidesPosition.MEDIUM);
        } else if (aimpad.isDPadUpPressed()) {
            setAutoSlidesPosition(AutoSlidesPosition.LONG);
        } else if (aimpad.isYPressed()) {
            setActiveControlState(SlidesControlState.MANUAL);
        }
        loop(aimpad);
        telemetry(telemetry);
    }

    private static final IntakeSystem.AutoSlidesPosition[] SLIDE_POSITIONS = {
            IntakeSystem.AutoSlidesPosition.RESET,
            IntakeSystem.AutoSlidesPosition.SHORT,
            IntakeSystem.AutoSlidesPosition.MEDIUM,
            IntakeSystem.AutoSlidesPosition.LONG
    };

    public IntakeSystem.AutoSlidesPosition getNextSlidePosition(IntakeSystem.AutoSlidesPosition currentPosition) {
        if (currentPosition == IntakeSystem.AutoSlidesPosition.LONG) {
            return IntakeSystem.AutoSlidesPosition.LONG;
        }
        int currentIndex = java.util.Arrays.asList(SLIDE_POSITIONS).indexOf(currentPosition);
        int nextIndex = (currentIndex + 1) % SLIDE_POSITIONS.length;
        return SLIDE_POSITIONS[nextIndex];
    }

    public IntakeSystem.AutoSlidesPosition getPreviousSlidePosition(IntakeSystem.AutoSlidesPosition currentPosition) {
        if (currentPosition == IntakeSystem.AutoSlidesPosition.RESET) {
            return IntakeSystem.AutoSlidesPosition.RESET;
        }
        int currentIndex = java.util.Arrays.asList(SLIDE_POSITIONS).indexOf(currentPosition);
        int previousIndex = (currentIndex - 1 + SLIDE_POSITIONS.length) % SLIDE_POSITIONS.length;
        return SLIDE_POSITIONS[previousIndex];
    }

}
