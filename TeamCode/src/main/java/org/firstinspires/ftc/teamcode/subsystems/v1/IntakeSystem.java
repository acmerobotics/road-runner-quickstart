package org.firstinspires.ftc.teamcode.subsystems.v1;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.subsystems.generic.SlidesBase;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.Hand;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.MultiAxisArm;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.Wrist;
import org.firstinspires.ftc.teamcode.util.ServoState;
import org.firstinspires.ftc.teamcode.util.StateDrivenServo;

public class IntakeSystem extends Mechanism {

    public MultiAxisArm multiAxisArm;
    public SlidesBase intakeSlides;

    private final DcMotorSimple.Direction leftMotorDirection = DcMotorSimple.Direction.REVERSE;
    private final DcMotorSimple.Direction rightMotorDirection = DcMotorSimple.Direction.FORWARD;
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

    StateDrivenServo leftSlidePivot;
    StateDrivenServo rightSlidePivot;

    ServoState PIVOT_UP = new ServoState(0);
    ServoState PIVOT_DOWN = new ServoState(1);

    public static final double RESET_POS = 0;
    public static final double SHORT_POS = 500;
    public static final double MEDIUM_POS = 1000;
    public static final double LONG_POS = 1600;

    public enum SlidesPosition {
        RESET, SHORT, MEDIUM, LONG
    }
    public SlidesPosition activeSlidesPosition = SlidesPosition.RESET;

    @Override
    public void init(HardwareMap hwMap) {
        multiAxisArm = new MultiAxisArm();
        multiAxisArm.init(hwMap);

        intakeSlides = new SlidesBase(ConfigurationInfo.leftIntakeSlide.getDeviceName(), ConfigurationInfo.rightIntakeSlide.getDeviceName(),
                leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        intakeSlides.init(hwMap);

        leftSlidePivot = new StateDrivenServo(new ServoState[]{PIVOT_UP, PIVOT_DOWN}, PIVOT_UP, ConfigurationInfo.leftIntakeSlidePivot.getDeviceName());
        rightSlidePivot = new StateDrivenServo(new ServoState[]{PIVOT_UP, PIVOT_DOWN}, PIVOT_UP, ConfigurationInfo.rightIntakeSlidePivot.getDeviceName());
        leftSlidePivot.init(hwMap);
        rightSlidePivot.init(hwMap);
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
        multiAxisArm.loop(aimpad);
        leftSlidePivot.loop(aimpad);
        rightSlidePivot.loop(aimpad);
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

    public void pivotUp() {
        leftSlidePivot.setActiveTargetState(PIVOT_UP);
        rightSlidePivot.setActiveTargetState(PIVOT_UP);
    }

    public void pivotDown() {
        leftSlidePivot.setActiveTargetState(PIVOT_DOWN);
        rightSlidePivot.setActiveTargetState(PIVOT_DOWN);
    }

    public void pivotCustom(double position) {
        leftSlidePivot.setActiveStateCustom(position);
        rightSlidePivot.setActiveStateCustom(position);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
    }

    /**
     * Method to individually check each hardware component of the intake system
     * @param aimpad the gamepad used to control the intake system
     */
    public void systemsCheck(AIMPad aimpad, AIMPad aimpad2, Telemetry telemetry) {
        if (aimpad.isDPadDownPressed()) {
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
        loop(aimpad, aimpad2);
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
