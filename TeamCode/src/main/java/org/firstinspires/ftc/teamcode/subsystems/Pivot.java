package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;

public class Pivot extends Mechanism {

    // ===============================================================
    // Constants and Control Variables
    // ===============================================================
    private static final double PROXIMITY_THRESHOLD = 1.0;  // Angle threshold for position accuracy
    private static final double MINIMUM_POWER = 0.03;       // Minimum power before holding position
    private static final double TICKS_PER_DEGREE = 15.4788888; // Encoder ticks per degree
    private static final int STARTING_DEGREES = 65; // Default starting angle

    private DcMotorEx pivotMotor;
    private PIDFController controller;
    private double lastAngle;
    private double activeTargetAngle = 0;
    private double manualPower = 0;
    private boolean hasSetHold = false;
    private boolean isFreeMovementEnabled = true;

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
    private static final double kP = 0.08;
    private static final double kI = 0;
    private static final double kD = 0.0008;
    private static final double INTEGRAL_SUM_MAX = 0;
    private static final double kV = 0.0;
    private static final double kA = 0.0;
    private static final double kStatic = 0.0;
    private static final double kCos = 0.02;
    private static final double kG = 0.0;

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
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize PIDF Controller
        controller = new PIDFController(kP, kI, kD, INTEGRAL_SUM_MAX, kV, kA, kStatic, kCos, kG, PIDFController.FeedforwardType.ROTATIONAL);

        // Set default pivot position
        setPivotPosition(PivotAngle.START);
        updateLastPosition();
    }

    // ===============================================================
    // Loop Control
    // ===============================================================
    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        if (!isFreeMovementEnabled) {
            if (!hasSetHold) {
                setTargetAngle(lastAngle);
                hasSetHold = true;
            }
            update();
        } else {
            hasSetHold = false;
            switch (activePivotControlState) {
                case AUTONOMOUS:
                    confirmPresetAngle();
                    update();
                    break;
                case MANUAL:
                    applyManualPower();
                    break;
            }
        }
        controller.setLength(slides.getCurrentExtension());
        isFreeMovementEnabled= slides.isPivotEnabled;
    }

    // ===============================================================
    // Telemetry for Debugging
    // ===============================================================
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Angle: ", getCurrentAngle());
        telemetry.addData("Target Angle: ", activeTargetAngle);
        telemetry.addData("Can Pivot: ", isFreeMovementEnabled);
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

    // ===============================================================
    // Manual Control Methods
    // ===============================================================

    private void applyManualPower() {
        if (Math.abs(manualPower) > MINIMUM_POWER) {
            setPower(manualPower);
        } else {
            holdAtCurrentAngle();
        }
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
