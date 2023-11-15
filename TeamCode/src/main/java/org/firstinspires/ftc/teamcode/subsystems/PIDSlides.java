package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.NoFeedback;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.LowPassEstimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

import java.util.function.DoubleSupplier;


public class PIDSlides extends Mechanism{
    private static final double KP = 0.001;
    private static final double KI = 0;
    private static final double KD = 0.0005;
    private static final double INTEGRAL_SUM_MAX = 0;
    private static final double STABILITY_THRESHOLD = 0;
    private static final double LOW_PASS_GAIN = 0;

    private static final double KV = 0;
    private static final double KA = 0;
    private static final double KSTATIC = 0;
    private static final double KG = 0;
    private static final double KCOS = 0;

    private static final double FILTER_LOW_PASS_GAIN = 0;
    PIDCoefficientsEx pidCoefficientsEx;
    PIDEx PIDController;

    FeedforwardCoefficientsEx feedforwardCoefficientsEx;
    FeedforwardEx FeedforwardController;

    final DoubleSupplier positionSupplier = this::getSlidesPosition;
    LowPassEstimator lowPassFilter;
    final RawValue noFilter = new RawValue(positionSupplier);
    final NoFeedback noFeedback = new NoFeedback();
    final NoFeedforward noFeedforward = new NoFeedforward();
    BasicSystem basicSystem;
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    private int lastEncoderMotorPos;
    double lastVelo;

    public double releaseSpeedLimit = 0.25;

    final String leftSlideName = "leftSlide";
    final String rightSlideName = "rightSlide";

    private boolean isSpeeding = false;

    private DcMotorEx encoderMotor;
    public double targetPos;

    public static final double PROXIMITY_THRESHOLD = 10;

    public static final int RESET_POS = 0;

    public static final int SAFE_EXTENSION_POS = -1900;
    public static final int SAFE_RETRACTION_POS = -1500;

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        pidCoefficientsEx = new PIDCoefficientsEx(KP, KI, KD, INTEGRAL_SUM_MAX, STABILITY_THRESHOLD, LOW_PASS_GAIN);
        PIDController = new PIDEx(pidCoefficientsEx);

        feedforwardCoefficientsEx = new FeedforwardCoefficientsEx(KV, KA, KSTATIC, KG, KCOS);
        FeedforwardController = new FeedforwardEx(feedforwardCoefficientsEx);

        lowPassFilter = new LowPassEstimator(positionSupplier, FILTER_LOW_PASS_GAIN);

        basicSystem = new BasicSystem(noFilter, PIDController, noFeedforward);
        encoderMotor = leftSlide;
    }

    @Override
    public void loop(Gamepad gamepad) {
        isSpeeding = getSlidesPower() > releaseSpeedLimit;
        lastVelo = getSlidesVelocity();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("leftSlidePos", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlidePos", rightSlide.getCurrentPosition());
        telemetry.addData("Target Position", targetPos);
        telemetry.addData("leftSlideVelo", leftSlide.getVelocity());
        telemetry.addData("rightSlideVelo", rightSlide.getVelocity());
        telemetry.addData("leftSlidePower", leftSlide.getPower());
        telemetry.addData("rightSlidePower", rightSlide.getPower());
        telemetry.addData("isSpeeding", isSpeeding);
        telemetry.addData("isAtTargetPosition", isAtTargetPosition());
    }

    public void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        setLastPosition();
    }

    public void stop() {
        setPower(0);
    }

    public void setMode(DcMotor.RunMode mode) {
        leftSlide.setMode(mode);
        rightSlide.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftSlide.setZeroPowerBehavior(behavior);
        rightSlide.setZeroPowerBehavior(behavior);
    }

    public void holdPosition() {
        update(getLastPosition());
    }

    private double setSlidesTargetPosition(double targetPosition) {
        return basicSystem.update(targetPosition);
    }

    public void update(double targetPosition) {
        double power = setSlidesTargetPosition(targetPosition);
        targetPos = targetPosition;
        setPower(power);
    }

    public int getSlidesPosition() {
        return encoderMotor.getCurrentPosition();
    }

    public double getSlidesVelocity() {
        return encoderMotor.getVelocity();
    }

    public double getSlidesPower() {
        return encoderMotor.getPower();
    }

    public void setLastPosition() {
        lastEncoderMotorPos = encoderMotor.getCurrentPosition();
    }

    public double getLastPosition() {
        return lastEncoderMotorPos;
    }

    public boolean isSpeeding() {
        return isSpeeding;
    }

    public boolean isAtTargetPosition() {
        return Math.abs(getSlidesPosition() - targetPos) < PROXIMITY_THRESHOLD;
    }
}