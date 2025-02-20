package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;

public class Pivot extends Mechanism {

    // ===============================================================
    // Constants and Control Variables
    // ===============================================================
    private static final double PROXIMITY_THRESHOLD = 1.2;  // Angle threshold for position accuracy
    private static final double MINIMUM_POWER = 0.03;       // Minimum power before holding position
    private static final double TICKS_PER_DEGREE = 15.4788888; // Encoder ticks per degree
    private static final int STARTING_DEGREES = 67; // Default starting angle
    private static final int MOVEMENT_PREVENTION_MILLIAMPS = 6000; // Prevents movement if current exceeds this value TODO FIND THIS THING PLEASE
    private static final int HIGH_PIDF_SWAP = 16;

    private DcMotorEx pivotMotor;
    private PIDFController activePIDF;
    private PIDFController controller;
    private PIDFController highController;
    private double lastAngle;
    private double activeTargetAngle = 0;
    private double manualPower = 0;

    // ===============================================================
    // Enums for Control State and Pivot Presets
    // ===============================================================
    public enum PivotControlState {
        AUTONOMOUS, MANUAL
    }

    public enum PivotAngle {
        START_MORE(STARTING_DEGREES - 20),
        START(STARTING_DEGREES),
        NEW_SCORE(89),
        SCORE(83),
        SPECIMEN_PICKUP(78),
        HIGH_BUCKET_RESET(92),
        CLIP_ON(130),
        HANG_OFF(110),
        PICKUP(173);

        public final int angle;

        PivotAngle(int angle) {
            this.angle = angle;
        }
    }

    private PivotControlState activePivotControlState = PivotControlState.AUTONOMOUS;
    private PivotAngle activePivotTarget = PivotAngle.START;

    // ===============================================================
    // PIDF Constants
    // ===============================================================
    private static final double kP = 0.115;
    private static final double kI = 0.005;
    private static final double kD = 0.01;
    private static final double INTEGRAL_SUM_MAX = 50;
    private static final double kV = 0.0;
    private static final double kA = 0.0;
    private static final double kStatic = 0.0;
    private static final double kCos = 0.01;
    private static final double kG = 0.0;

    private static final double highKP = .18;
    private static final double highKI = 0.01;
    private static final double highKD = 0.01;
    private static final double HIGH_INTEGRAL_SUM_MAX = 30;
    private static final double highKV = 0.0;
    private static final double highKA = 0.0;
    private static final double highKStatic = 0.0;
    private static final double highKCos = 0.02;
    private static final double highKG = 0.0;


    // ===============================================================
    // Constructor and Initialization
    // ===============================================================

    Slides slides;

    public Pivot(Slides slides) {
        this.slides = slides;
    }

    @Override
    public void init(HardwareMap hwMap) {
        // Initialize motor
        pivotMotor = hwMap.get(DcMotorEx.class, ConfigurationInfo.pivot.getDeviceName());
        pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize PIDF Controller
        controller = new PIDFController(kP, kI, kD, INTEGRAL_SUM_MAX, kV, kA, kStatic, kCos, kG, PIDFController.FeedforwardType.ROTATIONAL);
        highController = new PIDFController(highKP, highKI, highKD, HIGH_INTEGRAL_SUM_MAX, highKV, highKA, highKStatic, highKCos, highKG, PIDFController.FeedforwardType.ROTATIONAL);
        activePIDF = controller;
        // Set default pivot position
        setPivotPosition(PivotAngle.START);
        updateLastPosition();
    }

    // ===============================================================
    // Loop Control
    // ===============================================================
    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        switch (activePivotControlState) {
            case AUTONOMOUS:
                confirmPresetAngle();
                update();
                break;
            case MANUAL:
                applyManualPower();
                break;
        }
        updatePIDFController();
        activePIDF.setLength(slides.getCurrentExtension());
    }

    // ===============================================================
    // Telemetry for Debugging
    // ===============================================================
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Angle: ", getCurrentAngle());
        telemetry.addData("Target Angle: ", activeTargetAngle);
        telemetry.addData("Current Extension Feed Through:", slides.getCurrentExtension());
    }

    // ===============================================================
    // Core Motor Control Methods
    // ===============================================================

    private void updateLastPosition() {
        lastAngle = getCurrentAngle();
    }

    private void setPower(double power) {
        pivotMotor.setPower(power);
        updateLastPosition();
    }

    private double getTargetOutputPower() {
        return controller.calculateOutput(activeTargetAngle, getCurrentAngle(), 0, 0);
    }

    private void update() {
        double power = getTargetOutputPower();
        setPower(power);
    }

    private void setTargetAngle(double targetAngle) {
        activeTargetAngle = targetAngle;
        controller.setTargetPos(activeTargetAngle);
    }

    private void holdAtCurrentAngle() {
        setTargetAngle(lastAngle);
        update();
    }

    private void confirmPresetAngle() {
        if (activeTargetAngle != activePivotTarget.angle) {
            setTargetAngle(activePivotTarget.angle);
        }
    }

    public void setMode(DcMotor.RunMode mode) {
        pivotMotor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        pivotMotor.setZeroPowerBehavior(behavior);
    }

    // ===============================================================
    // Manual Control Methods
    // ===============================================================

    private void applyManualPower() {
        setPower(manualPower);
//        if (Math.abs(manualPower) > MINIMUM_POWER) {
//        } else {
//            holdAtCurrentAngle();
//        }
    }

    public void updateManualPower(double power) {
        manualPower = power;
    }

    public void setActiveControlState(PivotControlState state) {
        this.activePivotControlState = state;
    }

    // ===============================================================
    // High-Level Pivot Commands (Presets and Manual Override)
    // ===============================================================

    public boolean isAtTargetAngle() {
        return Math.abs(getCurrentAngle() - activeTargetAngle) < PROXIMITY_THRESHOLD;
    }

    public boolean isAtTargetPreset() {
        return Math.abs(getCurrentAngle() - activePivotTarget.angle) < PROXIMITY_THRESHOLD;
    }

    public void setPivotPosition(PivotAngle activePivotTarget) {
        setTargetAngle(activePivotTarget.angle);
        setActiveControlState(PivotControlState.AUTONOMOUS);
        this.activePivotTarget = activePivotTarget;
    }

    public void setPivotAtPower(double power) {
        updateManualPower(power);
        setActiveControlState(PivotControlState.MANUAL);
    }

    public boolean isMovementPrevented() {
        return pivotMotor.getCurrent(CurrentUnit.MILLIAMPS) > MOVEMENT_PREVENTION_MILLIAMPS && !isAtTargetPreset();
    }

    public void updatePIDFController() {
        if (slides.getCurrentExtension() > HIGH_PIDF_SWAP) {
            activePIDF = highController;
        } else {
            activePIDF = controller;
        }
    }

    // ===============================================================
    // Encoder Angle Conversion Methods
    // ===============================================================

    private double getCurrentAngle() {
        return ticksToDegrees(pivotMotor.getCurrentPosition()) + STARTING_DEGREES;
    }

    private double degreesToTicks(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }

    private double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }
}
