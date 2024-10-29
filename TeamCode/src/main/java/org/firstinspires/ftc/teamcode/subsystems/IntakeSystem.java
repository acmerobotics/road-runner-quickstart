package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

public class IntakeSystem extends Mechanism {

    Intake intake;
    SlidesBase intakeSlides;


    private static final DcMotorSimple.Direction leftMotorDirection = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction rightMotorDirection = DcMotorSimple.Direction.FORWARD;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double derivativeLowPassGain = 0.0;
    private static final double integralSumMax = 0.0;
    private static final double kV = 0.0;
    private static final double kA = 0.0;
    private static final double kStatic = 0.0;
    private static final double kCos = 0.0;
    private static final double kG = 0.0;
    private static final double lowPassGain = 0.0;

    Servo leftSlidePivot;
    Servo rightSlidePivot;

    private final static double DOWN_HINGE_POSITION = -0.5;
    private final static double UP_HINGE_POSITION = 0.5;

    public final double SLIDES_RESET_POS = 0;
    public final double SLIDES_LOW_CLIP_POS = 200;
    public final double SLIDES_HIGH_CLIP_POS = 400;
    public final double SLIDES_LOW_BUCKET_POS = 600;
    public final double SLIDES_HIGH_BUCKET_POS = 800;
    private double pivotTargetPosition = UP_HINGE_POSITION;

    public enum PivotState {
        DOWN, UP, CUSTOM
    }

    private PivotState activePivotState = PivotState.DOWN;

    @Override
    public void init(HardwareMap hwMap) {
        intake = new Intake();
        intake.init(hwMap);

        intakeSlides = new SlidesBase(ConfigurationInfo.leftIntakeSlide.getDeviceName(), ConfigurationInfo.rightIntakeSlide.getDeviceName(), leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        intakeSlides.init(hwMap);

        leftSlidePivot = hwMap.get(Servo.class, ConfigurationInfo.leftIntakeSlidePivot.getDeviceName());
        rightSlidePivot = hwMap.get(Servo.class, ConfigurationInfo.rightIntakeSlidePivot.getDeviceName());
    }

    @Override
    public void loop(AIMPad aimpad) {
        switch (activePivotState) {
            case DOWN:
                downState();
                break;
            case UP:
                upState();
                break;
            case CUSTOM:
                break;
        }
        pivotToPosition(pivotTargetPosition);
    }

    public void setActivePivotState(PivotState activePivotState) {
        this.activePivotState = activePivotState;
    }

    public void setPivotStateCustom(double position) {
        activePivotState = PivotState.CUSTOM;
        pivotTargetPosition = position;
    }

    public void downState() {
        pivotTargetPosition = DOWN_HINGE_POSITION;
    }

    public void upState() {
        pivotTargetPosition = UP_HINGE_POSITION;
    }

    public void pivotToPosition(double pivotPosition) {
        double clampedPivotPosition = Math.max(DOWN_HINGE_POSITION, Math.min(pivotPosition, UP_HINGE_POSITION));
        leftSlidePivot.setPosition(clampedPivotPosition);
        rightSlidePivot.setPosition(clampedPivotPosition);
    }

}
