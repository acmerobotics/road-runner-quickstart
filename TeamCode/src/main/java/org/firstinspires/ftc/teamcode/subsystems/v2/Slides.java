package org.firstinspires.ftc.teamcode.subsystems.v2;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.subsystems.generic.SlidesBase;
import org.firstinspires.ftc.teamcode.subsystems.v1.IntakeSystem;

public class Slides extends Mechanism {

    SlidesBase slides;

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

    public static final double RESET_POS = 0;
    public static final double LOW_POS = 500;
    public static final double MEDIUM_POS = 1000;
    public static final double HIGH_POS = 1600;

    enum SlidesPosition {
        RESET, LOW, MEDIUM, HIGH
    }

    public Slides.SlidesPosition activeSlidesPosition = Slides.SlidesPosition.RESET;

    DcMotorEx pivot;

    @Override
    public void init(HardwareMap hwMap) {
        slides = new SlidesBase(ConfigurationInfo.leftIntakeSlide.getDeviceName(), ConfigurationInfo.rightIntakeSlide.getDeviceName(),
            leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        slides.init(hwMap);
        pivot = hwMap.get(DcMotorEx.class, ConfigurationInfo.pivot.getDeviceName());
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        switch (activeSlidesPosition) {
            case LOW:
                lowState();
                break;
            case MEDIUM:
                mediumState();
                break;
            case HIGH:
                highState();
                break;
            case RESET:
                resetState();
                break;
        }
        slides.loop(aimpad, aimpad2);
    }

    public void setSlidesPosition(Slides.SlidesPosition activeSlidesPosition) {
        slides.setActiveControlState(SlidesBase.SlidesControlState.AUTONOMOUS);
        this.activeSlidesPosition = activeSlidesPosition;
    }

    public void lowState() {
        slides.setTargetPosition(LOW_POS);
    }

    public void mediumState() {
        slides.setTargetPosition(MEDIUM_POS);
    }

    public void highState() {
        slides.setTargetPosition(HIGH_POS);
    }

    public void resetState() {
        slides.setTargetPosition(RESET_POS);
    }
}
