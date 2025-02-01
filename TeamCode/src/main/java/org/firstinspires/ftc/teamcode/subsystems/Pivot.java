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

    private static final double PROXIMITY_THRESHOLD = 20;
    private DcMotorEx pivot;

    private final SimpleControlSystem controlSystem;
  
    public enum PivotControlState {
        AUTONOMOUS, MANUAL
    }

    private PivotControlState activePivotControlState = PivotControlState.AUTONOMOUS;
  
    private double lastPosition;

    private double activeTargetPosition = 0;

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

    public enum PivotPosition {
        PICKUP(300),
        SCORE(100),
        HANG(0);

        private final int position;

        PivotPosition(int position) {
            this.position = position;
        }
    }


    private static final double TICKS_PER_DEGREE = 1000; //TODO set

    private static final int STARTING_DEGREES = 10;
    private PivotPosition activePivotPosition = PivotPosition.PICKUP;
  
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
        setPivotPosition(PivotPosition.DOWN);
        updateLastPosition();
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        switch (activePivotControlState) {
            case AUTONOMOUS:
                update();
                break;
            case MANUAL:
                applyManualPower();
                break;
        }

    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current Position: ", pivot.getCurrentPosition());
        telemetry.addData("Target Position: ", activeTargetPosition);
    }


    private void updateLastPosition() {
        lastPosition = pivot.getCurrentPosition();
    }
      
    private void setPower(double power) {
        pivot.setPower(power);
        updateLastPosition();
    }

    private double getTargetOutputPower() {
        return controlSystem.update(pivot.getCurrentPosition() + degreesToTicks(STARTING_DEGREES));
    }
      
    private void update() {
        double power = getTargetOutputPower();
        setPower(power);
    }

    private void setTargetPosition(double targetPosition) {
        activeTargetPosition = degreesToTicks(targetPosition);
        controlSystem.setTarget(activeTargetPosition);
    }
      
    private void holdPosition() {
        setTargetPosition(lastPosition);
        update();
    }

    private void applyManualPower() {
        if (Math.abs(manualPower) > MINIMUM_POWER) {
            setPower(manualPower);
        } else {
            holdPosition();
        }
    }

    private void updateManualPower(double power) {
        manualPower = power;
    }

    private void setActiveControlState(PivotControlState activeControlState) {
        this.activePivotControlState = activeControlState;
    }

    /**
     * Check if the slides are at the target position
     * @return true if the slides are within the proximity threshold of the target position
     */
    public boolean isAtTargetPosition() {
        return Math.abs(pivot.getCurrentPosition() - activeTargetPosition) < PROXIMITY_THRESHOLD;
    }

    public void setPivotPosition(PivotPosition activePivotPosition) {
        setTargetPosition(activePivotPosition.position);
        setActiveControlState(PivotControlState.AUTONOMOUS);
        this.activePivotPosition = activePivotPosition;
    }

    public void setPivotAtPower(double power) {
        setActiveControlState(PivotControlState.MANUAL);
        updateManualPower(power);
    }

    public double degreesToTicks(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }
}