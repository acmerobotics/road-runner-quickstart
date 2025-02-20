package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.control.FeedforwardController;
import com.aimrobotics.aimlib.control.LowPassFilter;
import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.control.SimpleControlSystem;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.control.PIDFController;

public class Slides extends Mechanism {

    // ===============================================================
    // Enums for control state and preset slide extensions
    // ===============================================================
    public enum SlidesControlState {
        AUTONOMOUS, MANUAL
    }

    public enum SlidesExtension {
        RESET_MORE(-50),
        RESET(0),
        HIGH_SPECIMEN(5.75),
        HIGH_SPECIMEN_AUTO(7),
        LOW_BUCKET(14),
        LOW_HANG(19),
        HIGH_HANG(23),
        NEXT_HANG(6),
        HIGH_BUCKET(28.5);

        public final double extension;

        SlidesExtension(double extension) {
            this.extension = extension;
        }
    }

    // ===============================================================
    // Motor and encoder fields
    // ===============================================================
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    private DcMotorEx activeEncoderMotor;
    private double lastActiveEncoderExtension;

    // ===============================================================
    // Motor configuration names and directions
    // ===============================================================
    // (Obtained from ConfigurationInfo in init())
    private String leftSlideName;
    private String rightSlideName;
    private final DcMotorSimple.Direction leftMotorDirection = DcMotorSimple.Direction.REVERSE;
    private final DcMotorSimple.Direction rightMotorDirection = DcMotorSimple.Direction.FORWARD;

    // ===============================================================
    // Control system and parameters (PID, feedforward, filtering)
    // ===============================================================
    private PIDFController controller;

    // PID and feedforward constants (set to your desired values)
    private static final double kP = 0.5;
    private static final double kI = 0.03;
    private static final double kD = 0;
    private static final double INTEGRAL_SUM_MAX = 10;
    private static final double kV = 0;
    private static final double kA = 0;
    private static final double kStatic = 0;
    private static final double kCos = 0;
    private static final double kG = 0;

    // ===============================================================
    // Control variables
    // ===============================================================
    protected double activeTargetExtension = 0;
    private double manualPower = 0;
    protected SlidesControlState activeControlState = SlidesControlState.AUTONOMOUS;

    // ===============================================================
    // Constants for motion and safety
    // ===============================================================
    private static final double PROXIMITY_THRESHOLD = .75;   // Inches tolerance for target extension
    private static final double CURRENT_THRESHOLD = 5000;     // In milliamps
    private static final double MINIMUM_POWER = 0.03;           // Minimum manual power to move slides
    private static final double TICKS_PER_INCH = 113.91949;     // Encoder ticks per inch of slide travel
    private static final double EXTEND_MAX = 17;

    // ===============================================================
    // Additional fields for high-level control (preset positions, pivot)
    // ===============================================================
    public SlidesExtension activeSlidesTarget = SlidesExtension.RESET;

    // ===============================================================
    // Initialization method
    // ===============================================================
    @Override
    public void init(HardwareMap hwMap) {
        // Retrieve motor names from your configuration (adjust as needed)
        leftSlideName = ConfigurationInfo.leftSlide.getDeviceName();
        rightSlideName = ConfigurationInfo.rightSlide.getDeviceName();

        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);

        // Reset encoders and set motor modes
        setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        leftSlide.setDirection(leftMotorDirection);
        rightSlide.setDirection(rightMotorDirection);

        // Use one motor as the primary encoder source (.                                                                        can be adjusted as needed)
        activeEncoderMotor = rightSlide;
        lastActiveEncoderExtension = 0;

        controller = new PIDFController(kP, kI, kD, INTEGRAL_SUM_MAX, kV, kA, kStatic, kCos, kG, PIDFController.FeedforwardType.LINEAR);
    }

    // ===============================================================
    // Main loop: choose autonomous (closed-loop) or manual (open-loop) control
    // ===============================================================
    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        // Control the slides based on the active control state
        switch (activeControlState) {
            case AUTONOMOUS:
                updateAutonomous();
                break;
            case MANUAL:
                applyManualPower();
                break;
        }
    }

    // ===============================================================
    // Telemetry reporting method
    // ===============================================================
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Position (ticks):", getCurrentPosition());
        telemetry.addData("Current Extension (in):", getCurrentExtension());
        telemetry.addData("Target Extension (in):", activeTargetExtension);
        telemetry.addData("Control Mode:", activeControlState);
        telemetry.addData("MILLIAMPS", activeEncoderMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }

    // ===============================================================
    // Low-level motor control methods (similar to SlidesBase)
    // ===============================================================

    /**
     * Set the mode of both slide motors.
     */
    public void setMode(DcMotorEx.RunMode mode) {
        leftSlide.setMode(mode);
        rightSlide.setMode(mode);
    }

    /**
     * Set the zero power behavior of both slide motors.
     */
    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        leftSlide.setZeroPowerBehavior(behavior);
        rightSlide.setZeroPowerBehavior(behavior);
    }

    /**
     * Update the autonomous (closed-loop) control.
     */
    private void updateAutonomous() {
        double power = getTargetOutputPower();
        setPower(power);
    }

    /**
     * Convert the current extension (in inches) into a power output via the control system.
     */
    private double getTargetOutputPower() {
        return controller.calculateOutput(activeTargetExtension, getCurrentExtension(), 0, 0);
    }

    /**
     * Sets the power for both slide motors and updates the last known extension.
     */
    private void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        updateLastExtension();
    }

    /**
     * Hold the current position by setting the target to the last known extension.
     */
    private void holdPosition() {
        setTargetExtension(getLastExtension());
        updateAutonomous();
    }

    /**
     * Set a new target extension (in inches) for the slides.
     */
    public void setTargetExtension(double targetExtension) {
        activeTargetExtension = targetExtension;
        controller.setTargetPos(activeTargetExtension);
    }

    /**
     * Returns true if the current extension is within the proximity threshold of the target.
     */
    public boolean isAtTargetExtension() {
        return Math.abs(getCurrentExtension() - activeTargetExtension) < PROXIMITY_THRESHOLD;
    }

    /**
     * Check if a current spike is detected.
     */
    public boolean currentSpikeDetected() {
        return activeEncoderMotor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD;
    }

    /**
     * Returns the raw encoder position from the active motor.
     */
    public double getCurrentPosition() {
        return activeEncoderMotor.getCurrentPosition();
    }

    /**
     * Returns the current slide extension in inches.
     */
    public double getCurrentExtension() {
        return ticksToInches(getCurrentPosition());
    }

    /**
     * Returns the current velocity (in ticks per second) of the active motor.
     */
    public double getCurrentVelocity() {
        return activeEncoderMotor.getVelocity();
    }

    /**
     * Returns the current velocity converted to inches per second.
     */
    public double getCurrentVelocityInchesPerSecond() {
        return ticksToInches(getCurrentVelocity());
    }

    /**
     * Update the last known encoder extension (in inches).
     */
    private void updateLastExtension() {
        lastActiveEncoderExtension = ticksToInches(getCurrentPosition());
    }

    /**
     * Get the last known extension.
     */
    private double getLastExtension() {
        return lastActiveEncoderExtension;
    }

    /**
     * Convert encoder ticks to inches.
     */
    private double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    // ===============================================================
    // Manual control methods
    // ===============================================================

    /**
     * Sets the manual power for the slides. If the power is above the minimum threshold
     * and no current spike is detected, the power is applied; otherwise, the slides hold position.
     */
    private void applyManualPower() {
        if (Math.abs(manualPower) > MINIMUM_POWER && !currentSpikeDetected()) {
             if (getCurrentExtension() > EXTEND_MAX) {
                 setPower(-1);
             } else  {
                 setPower(manualPower);
             }
        } else {
            holdPosition();
        }
    }

    /**
     * Update the manual power variable.
     */
    public void updateManualPower(double power) {
        manualPower = power;
    }

    /**
     * Set the control mode.
     */
    public void setActiveControlState(SlidesControlState state) {
        activeControlState = state;
    }

    // ===============================================================
    // High-level slide commands (using presets and manual overrides)
    // ===============================================================

    /**
     * Set the slide to a preset position.
     */
    public void setSlidesPosition(SlidesExtension targetPreset) {
        setTargetExtension(targetPreset.extension);
        setActiveControlState(SlidesControlState.AUTONOMOUS);
        activeSlidesTarget = targetPreset;
    }

    /**
     * Manually drive the slides at the specified power.
     */
    public void setSlidesAtPower(double power) {
        setActiveControlState(SlidesControlState.MANUAL);
        updateManualPower(power);
    }

    /**
     * Returns true if the slide is at the target extension.
     */
    public boolean isAtTargetPosition() {
        return isAtTargetExtension();
    }

    /**
     * Returns true if the current extension is within the proximity threshold of the preset.
     */
    public boolean isAtTargetPreset() {
        return Math.abs(getCurrentExtension() - activeSlidesTarget.extension) < PROXIMITY_THRESHOLD;
    }
}