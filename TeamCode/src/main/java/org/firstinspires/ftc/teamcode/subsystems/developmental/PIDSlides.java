package org.firstinspires.ftc.teamcode.subsystems.developmental;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.NoFeedback;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.LowPassEstimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

import org.firstinspires.ftc.teamcode.util.RunToPositionMotorUtil;

import java.util.function.DoubleSupplier;


public class PIDSlides extends Mechanism{
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double INTEGRAL_SUM_MAX = 0;
    private static final double STABILITY_THRESHOLD = 0;
    private static final double LOW_PASS_GAIN = 0;

    private static final double KV = 0;
    private static final double KA = 0;
    private static final double KSTATIC = 0;
    private static final double KG = 0;
    private static final double KCOS = 0;

    private static final double FILTER_LOW_PASS_GAIN = 0;
    private PIDCoefficientsEx pidCoefficientsEx;
    private PIDEx PIDController;

    private FeedforwardCoefficientsEx feedforwardCoefficientsEx;
    private FeedforwardEx FeedforwardController;

    private final DoubleSupplier positionSupplier = this::getSlidesPosition;
    private LowPassEstimator lowPassFilter;
    private final RawValue noFilter = new RawValue(positionSupplier);
    private final NoFeedback noFeedback = new NoFeedback();
    private final NoFeedforward noFeedforward = new NoFeedforward();
    private BasicSystem basicSystem;
    public boolean isResettingLifts = false;

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    private int lastEncoderMotorPos;
    private double lastVelo;

    public double releaseSpeedLimit = 0.25;

    private final String leftSlideName = "leftSlide";
    private final String rightSlideName = "rightSlide";

    private boolean isSpeeding = false;
    private boolean isReset = true;

    private final DcMotorEx encoderMotor;

    PIDSlides(DcMotorEx encoderMotor) {
        this.encoderMotor = encoderMotor;
    }

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        pidCoefficientsEx = new PIDCoefficientsEx(KP, KI, KD, INTEGRAL_SUM_MAX, STABILITY_THRESHOLD, LOW_PASS_GAIN);
        PIDController = new PIDEx(pidCoefficientsEx);

        feedforwardCoefficientsEx = new FeedforwardCoefficientsEx(KV, KA, KSTATIC, KG, KCOS);
        FeedforwardController = new FeedforwardEx(feedforwardCoefficientsEx);

        lowPassFilter = new LowPassEstimator(positionSupplier, FILTER_LOW_PASS_GAIN);

        basicSystem = new BasicSystem(noFilter, PIDController, noFeedforward);
    }

    @Override
    public void loop(Gamepad gamepad) {
        isSpeeding = getSlidesPower() > releaseSpeedLimit;
        isReset = getLastPosition() == 0;
        lastVelo = getSlidesVelocity();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
//        telemetry.addData("kG", kG);
        telemetry.addData("leftSlidePos", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlidePos", rightSlide.getCurrentPosition());
        telemetry.addData("leftSlideVelo", leftSlide.getVelocity());
        telemetry.addData("rightSlideVelo", rightSlide.getVelocity());
        telemetry.addData("leftSlidePower", leftSlide.getPower());
        telemetry.addData("rightSlidePower", rightSlide.getPower());
        telemetry.addData("isSpeeding", isSpeeding);
        telemetry.addData("isReset", isReset);
        telemetry.addData("isResettingLifts", isResettingLifts);
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
        double power = basicSystem.update(getLastPosition());
        setPower(power);
    }

    public void setSlidesTargetPosition(int targetPosition) {
        basicSystem.update(targetPosition);
    }
    public void resetSlidesPosition() {
        isResettingLifts = true;
        setSlidesTargetPosition(0);
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

    public boolean isReset() {
        return isReset;
    }

    public boolean isSpeeding() {
        return isSpeeding;
    }
}