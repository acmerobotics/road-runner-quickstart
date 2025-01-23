package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.control.FeedforwardController;
import com.aimrobotics.aimlib.control.LowPassFilter;
import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.control.SimpleControlSystem;
import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Pivot extends Mechanism {
    private DcMotorEx pivot;

    private final SimpleControlSystem controlSystem;
    public enum PivotControlState {
        AUTONOMOUS, MANUAL
    }

    private SlidesBase.SlidesControlState activeControlState = SlidesBase.SlidesControlState.AUTONOMOUS;
    private double lastPosition;
    private double activeTargetPosition = 0;
    private static final double MINIMUM_POWER = 0.03;
    private double manualPower = 0;



    //todo: set pid values

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

    /**
     * Constructor for the slides base
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param derivativeLowPassGain the derivative low pass gain
     * @param integralSumMax the maximum integral sum
     * @param kV the velocity feedforward gain
     * @param kA  the acceleration feedforward gain
     * @param kStatic the static feedforward gain
     * @param kCos the cosine feedforward gain (Only this or kG should be used)
     * @param kG the gravity feedforward gain (Only this or kCos should be used)
     * @param lowPassGain the low pass gain
     */
    public Pivot(double kP, double kI, double kD, double derivativeLowPassGain, double integralSumMax, double kV, double kA, double kStatic, double kCos, double kG, double lowPassGain) {
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
        lastPosition = 0;
    }

    @Override
    public void loop(AIMPad aimpad){
        switch (activeControlState){
            case AUTONOMOUS:
                update();
                break;
            case MANUAL:
                applyManualPower();
                break;
        }

    }


    private void updateLastPosition() {
        lastPosition = pivot.getCurrentPosition();
    }
    private void setPower(double power) {
        pivot.setPower(power);
        updateLastPosition();
    }

    private double getTargetOutputPower() {
        return controlSystem.update(pivot.getCurrentPosition());
    }
    private void update() {
        double power = getTargetOutputPower();
        setPower(power);
    }

    public void setTargetPosition(double targetPosition) {
        activeTargetPosition = targetPosition;
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



}
