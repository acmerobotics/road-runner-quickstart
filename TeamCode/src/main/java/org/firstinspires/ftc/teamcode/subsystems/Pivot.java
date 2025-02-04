package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.control.FeedforwardController;
import com.aimrobotics.aimlib.control.LowPassFilter;
import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.control.SimpleControlSystem;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Pivot extends Mechanism {

    private static final double PROXIMITY_THRESHOLD = 2;
    private DcMotorEx pivot;

    private final SimpleControlSystem controlSystem;
  
    public enum PivotControlState {
        AUTONOMOUS, MANUAL
    }

    private PivotControlState activePivotControlState = PivotControlState.AUTONOMOUS;
  
    private double lastAngle;

    private double activeTargetAngle = 0;

    private static final double MINIMUM_POWER = 0.03;
    private double manualPower = 0;

    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double derivativeLowPassGain = 0;
    private static final double integralSumMax = 0;
    private static final double kV = 0.0;
    private static final double kA = 0.0;
    private static final double kStatic = 0.0;
    private static final double kCos = 0.05;
    private static final double kG = 0.0;
    private static final double lowPassGain = 0;

    public enum PivotAngle {
        PICKUP(300),
        SCORE(100),
        HANG(0);

        private final int angle;

        PivotAngle(int angle) {
            this.angle = angle;
        }
    }


    private static final double TICKS_PER_DEGREE = 3.95861111111;

    private static final int STARTING_DEGREES = 10;

    private PivotAngle activePivotTarget = PivotAngle.SCORE;

    private boolean isFreeMovementEnabled = true;
  
    public Pivot() {
        PIDController pidController = new PIDController(kP, kI, kD, derivativeLowPassGain, integralSumMax);
        FeedforwardController feedforwardController = new FeedforwardController(kV, kA, kStatic, kCos, kG);
        LowPassFilter lowPassFilter = new LowPassFilter(lowPassGain);
        controlSystem = new SimpleControlSystem(pidController, feedforwardController, lowPassFilter);
    }


    @Override
    public void init(HardwareMap hwMap) {
        pivot = hwMap.get(DcMotorEx.class, ConfigurationInfo.pivot.getDeviceName());
        pivot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        setPivotPosition(PivotAngle.PICKUP);
        updateLastPosition();
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        if (!isFreeMovementEnabled) {
            holdAtCurrentAngle();
        } else {
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

    }

    public void setIsFreeMovementEnabled(boolean condition) {
        isFreeMovementEnabled = condition;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Angle: ", getCurrentAngle());
        telemetry.addData("Target Angle: ", activeTargetAngle);
    }

    private void updateLastPosition() {
        lastAngle = getCurrentAngle();
    }
      
    private void setPower(double power) {
        pivot.setPower(power);
        updateLastPosition();
    }

    private double getTargetOutputPower() {
        return controlSystem.update(getCurrentAngle() + STARTING_DEGREES);
    }
      
    private void update() {
        double power = getTargetOutputPower();
        setPower(power);
    }

    private void setTargetAngle(double targetAngle) {
        activeTargetAngle = targetAngle;
        controlSystem.setTarget(activeTargetAngle);
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

    private void applyManualPower() {
        if (Math.abs(manualPower) > MINIMUM_POWER) {
            setPower(manualPower);
        } else {
            holdAtCurrentAngle();
        }
    }

    private void updateManualPower(double power) {
        manualPower = power;
    }

    private void setActiveControlState(PivotControlState activeControlState) {
        this.activePivotControlState = activeControlState;
    }

    private double getCurrentAngle() {
        return ticksToDegrees(pivot.getCurrentPosition());
    }

    /**
     * Check if the pivot is at the target angle
     * @return true if the pivot is within the proximity threshold of the target angle
     */
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

    private double degreesToTicks(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }

    private double ticksToDegrees(double ticks) {
        return ticks * (1/TICKS_PER_DEGREE);
    }
}