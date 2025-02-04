package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Slides extends Mechanism {

    private SlidesBase slidesBase;

    private final DcMotorSimple.Direction leftMotorDirection = DcMotorSimple.Direction.REVERSE;
    private final DcMotorSimple.Direction rightMotorDirection = DcMotorSimple.Direction.REVERSE;
    private static final double kP = 0.006;
    private static final double kI = 0.0002;
    private static final double kD = 0.0001;
    private static final double derivativeLowPassGain = 0.1;
    private static final double integralSumMax = 2500;
    private static final double kV = 0;
    private static final double kA = 0;
    private static final double kStatic = 0;
    private static final double kCos = 0;
    private static final double kG = 0;
    private static final double lowPassGain = 0.1;

    public enum SlidesExtension {
        RESET(0),
        LOW_BUCKET(17),
        HIGH_BUCKET(32);

        private final double extension;

        SlidesExtension(double extension) {
            this.extension = extension;
        }
    }

    public SlidesExtension activeSlidesTarget = SlidesExtension.RESET;

    public boolean isPivotEnabled = true;
    private static final double PIVOT_ENABLED_THRESHOLD = 3;
    private static final double PROXIMITY_THRESHOLD = 0.25;

    @Override
    public void init(HardwareMap hwMap) {
        slidesBase = new SlidesBase(ConfigurationInfo.leftSlide.getDeviceName(), ConfigurationInfo.rightSlide.getDeviceName(),
            leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        slidesBase.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        slidesBase.loop(aimpad, aimpad2);
        isPivotEnabled = slidesBase.getCurrentPosition() < PIVOT_ENABLED_THRESHOLD;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Position: ", slidesBase.getCurrentPosition());
        telemetry.addData("Target Extension: ", slidesBase.activeTargetExtension);
        telemetry.addData("Mode: ", slidesBase.activeControlState);
    }

    public void setSlidesPosition(SlidesExtension activeSlidesTarget) {
        slidesBase.setTargetExtension(activeSlidesTarget.extension);
        slidesBase.setActiveControlState(SlidesBase.SlidesControlState.AUTONOMOUS);
        this.activeSlidesTarget = activeSlidesTarget;
    }

    public void setSlidesAtPower(double power) {
        slidesBase.setActiveControlState(SlidesBase.SlidesControlState.MANUAL);
        slidesBase.updateManualPower(power);
    }

    public boolean isAtTargetPosition() {
        return slidesBase.isAtTargetExtension();
    }

    public boolean isAtTargetPreset() {
        return Math.abs(slidesBase.getCurrentExtension() - activeSlidesTarget.extension) < PROXIMITY_THRESHOLD;
    }
}
