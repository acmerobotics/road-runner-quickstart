package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

public class OuttakeSystem extends Mechanism {
    Outtake outtake;
    public SlidesBase outtakeSlides;

    private static final DcMotorSimple.Direction leftMotorDirection = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction rightMotorDirection = DcMotorSimple.Direction.REVERSE;

    private static final double kP = 0.006;
    private static final double kI = 0.00001;
    private static final double kD = 0.00004;
    private static final double derivativeLowPassGain = 0;
    private static final double integralSumMax = 2500;
    private static final double kV = 0.01;
    private static final double kA = 0.0;
    private static final double kStatic = 0.0;
    private static final double kCos = 0.0;
    private static final double kG = 0;
    private static final double lowPassGain = 0;

    public static final double RESET_POS = 0;
    public static final double SHORT_POS = 1500;
    public static final double TALL_POS = 3100;
    public static final double SPECIMEN_LOW_POS = 650;
    public static final double SPECIMEN_LOW_DROP_POS = 350;
    public static final double SPECIMEN_HIGH_POS = 1850;
    public static final double SPECIMEN_HIGH_DROP_POS = 1350;

    public enum AutoSlidesPosition {
        RESET, SHORT, TALL, SPECIMEN_LOW, SPECIMEN_HIGH, SPECIMEN_LOW_DROP, SPECIMEN_HIGH_DROP
    }
    public OuttakeSystem.AutoSlidesPosition activeAutoSlidesPosition = OuttakeSystem.AutoSlidesPosition.RESET;

    public enum SlidesControlState {
        AUTONOMOUS, MANUAL
    }
    private OuttakeSystem.SlidesControlState activeControlState = OuttakeSystem.SlidesControlState.AUTONOMOUS;

    public double manualPower = 0;


    @Override
    public void init(HardwareMap hwMap) {
        outtake = new Outtake();
        outtake.init(hwMap);

        outtakeSlides = new SlidesBase(ConfigurationInfo.leftOuttakeSlide.getDeviceName(), ConfigurationInfo.rightOuttakeSlide.getDeviceName(), leftMotorDirection, rightMotorDirection, kP, kI, kD, derivativeLowPassGain, integralSumMax, kV, kA, kStatic, kCos, kG, lowPassGain);
        outtakeSlides.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
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
                    case SPECIMEN_LOW:
                        specimenLowState();
                        break;
                    case SPECIMEN_HIGH:
                        specimenHighState();
                        break;
                    case SPECIMEN_LOW_DROP:
                        specimenLowDropState();
                        break;
                    case SPECIMEN_HIGH_DROP:
                        specimenHighDropState();
                        break;
                }
                outtakeSlides.update();
                break;
            case MANUAL:
                manualMode();
                break;
        }
        outtake.loop(aimpad);
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

    /**
     * Set the slides to the specimen low position
     */
    public void specimenLowState() {
        outtakeSlides.setTargetPosition(SPECIMEN_LOW_POS);
    }

    /**
     * Set the slides to the specimen high position
     */
    public void specimenHighState() {
        outtakeSlides.setTargetPosition(SPECIMEN_HIGH_POS);
    }

    /**
     * Set the slides to the specimen low drop position
     */
    public void specimenLowDropState() {
        outtakeSlides.setTargetPosition(SPECIMEN_LOW_DROP_POS);
    }

    /**
     * Set the slides to the specimen high drop position
     */
    public void specimenHighDropState() {
        outtakeSlides.setTargetPosition(SPECIMEN_HIGH_DROP_POS);
    }

    public void manualMode() {
        if (Math.abs(manualPower) > GamepadSettings.GP1_STICK_DEADZONE) {
            outtakeSlides.setPower(manualPower);
        } else {
            outtakeSlides.holdPosition();
        }
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Slides Position", outtakeSlides.getCurrentPosition());
    }

    public void systemsCheck(AIMPad aimpad, Telemetry telemetry) {
        if (aimpad.isDPadDownPressed()) {
            setAutoSlidesPosition(OuttakeSystem.AutoSlidesPosition.RESET);
        } else if (aimpad.isDPadLeftPressed()) {
            setAutoSlidesPosition(OuttakeSystem.AutoSlidesPosition.SHORT);
        } else if (aimpad.isDPadUpPressed()) {
            setAutoSlidesPosition(OuttakeSystem.AutoSlidesPosition.TALL);
        } else if (aimpad.isYPressed()) {
            setActiveControlState(OuttakeSystem.SlidesControlState.MANUAL);
        }
        loop(aimpad);
        telemetry(telemetry);
    }

}
