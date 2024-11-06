package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

public class OuttakeSystem extends Mechanism {
    Outtake outtake;
    SlidesBase outtakeSlides;

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

    public static final double RESET_POS = 0;
    public static final double SHORT_POS = 600;
    public static final double TALL_POS = 800;

    private enum AutoSlidesPosition {
        RESET, SHORT, TALL
    }
    private OuttakeSystem.AutoSlidesPosition activeAutoSlidesPosition = OuttakeSystem.AutoSlidesPosition.RESET;

    private enum SlidesControlState {
        AUTONOMOUS, MANUAL
    }
    private OuttakeSystem.SlidesControlState activeControlState = OuttakeSystem.SlidesControlState.AUTONOMOUS;


    @Override
    public void init(HardwareMap hwMap) {
        outtake = new Outtake();
        outtake.init(hwMap);

        outtakeSlides = new SlidesBase(ConfigurationInfo.leftOuttakeSlide.getDeviceName(), ConfigurationInfo.rightOuttakeSlide.getDeviceName(), leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        outtakeSlides.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad) {
        switch (activeControlState){
            case AUTONOMOUS:
                switch(activeAutoSlidesPosition) {
                    case RESET:
                        resetState();
                        break;
                    case SHORT:
                        shortState();
                        break;
                    case TALL:
                        tallState();
                        break;
                }
                outtakeSlides.update();
                break;
            case MANUAL:
                if (aimpad.isLeftStickMovementEngaged() && !outtakeSlides.currentSpikeDetected()) {
                    outtakeSlides.setPower(aimpad.getLeftStickY());
                } else {
                    outtakeSlides.holdPosition();
                }
                break;
        }
    }

    public void setActiveControlState(OuttakeSystem.SlidesControlState activeControlState) {
        this.activeControlState = activeControlState;
    }

    public void setAutoSlidesPosition(OuttakeSystem.AutoSlidesPosition activeAutoSlidesPosition) {
        setActiveControlState(OuttakeSystem.SlidesControlState.AUTONOMOUS);
        this.activeAutoSlidesPosition = activeAutoSlidesPosition;
    }

    /**
     * Set the slides to the reset position
     */
    public void resetState() {
        outtakeSlides.setTargetPosition(RESET_POS);
    }

    /**
     * Set the slides to the short position
     */
    public void shortState() {
        outtakeSlides.setTargetPosition(SHORT_POS);
    }

    /**
     * Set the slides to the tall position
     */
    public void tallState() {
        outtakeSlides.setTargetPosition(TALL_POS);
    }

    public void systemsCheck(AIMPad aimpad) {
        if (aimpad.isDPadDownPressed()) {
            setAutoSlidesPosition(OuttakeSystem.AutoSlidesPosition.RESET);
        } else if (aimpad.isDPadLeftPressed()) {
            setAutoSlidesPosition(OuttakeSystem.AutoSlidesPosition.SHORT);
        } else if (aimpad.isDPadUpPressed()) {
            setAutoSlidesPosition(OuttakeSystem.AutoSlidesPosition.TALL);
        } else if (aimpad.isLeftStickMovementEngaged()) {
            setActiveControlState(OuttakeSystem.SlidesControlState.MANUAL);
        }
        loop(aimpad);
    }

}
