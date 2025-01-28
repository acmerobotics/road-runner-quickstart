package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Slides extends Mechanism {

    private SlidesBase slides;

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

    enum SlidesPosition {
        RESET(0),
        LOW(500),
        MEDIUM(1000),
        HIGH(1600);

        private final int position;

        SlidesPosition(int position) {
            this.position = position;
        }
    }

    public Slides.SlidesPosition activeSlidesPosition = Slides.SlidesPosition.RESET;

    @Override
    public void init(HardwareMap hwMap) {
        slides = new SlidesBase(ConfigurationInfo.leftIntakeSlide.getDeviceName(), ConfigurationInfo.rightIntakeSlide.getDeviceName(),
            leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        slides.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        slides.loop(aimpad, aimpad2);
    }

    public void setSlidesPosition(Slides.SlidesPosition activeSlidesPosition) {
        slides.setTargetPosition(activeSlidesPosition.position);
        slides.setActiveControlState(SlidesBase.SlidesControlState.AUTONOMOUS);
        this.activeSlidesPosition = activeSlidesPosition;
    }

    public void setSlidesAtPower(double power) {
        slides.setActiveControlState(SlidesBase.SlidesControlState.MANUAL);
        slides.updateManualPower(power);
    }
}
