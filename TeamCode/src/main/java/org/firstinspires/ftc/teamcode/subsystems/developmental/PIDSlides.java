package org.firstinspires.ftc.teamcode.subsystems.developmental;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.LowPassEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Mechanism;

import org.firstinspires.ftc.teamcode.util.RunToPositionMotorUtil;

import java.util.function.DoubleSupplier;


public class PIDSlides extends Mechanism{
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double integralSumMax = 0;
    double stabilityThreshold = 0;
    double lowPassGain = 0;

    double kV = 0;
    double kA = 0;
    double kStatic = 0;
    double kG = 0;
    double kCos = 0;

    double a = 0;
    PIDCoefficientsEx pidCoefficientsEx = new PIDCoefficientsEx(kP, kI, kD, integralSumMax, stabilityThreshold, lowPassGain);
    PIDEx PIDController = new PIDEx(pidCoefficientsEx);

    FeedforwardCoefficientsEx feedforwardCoefficientsEx = new FeedforwardCoefficientsEx(kV, kA, kStatic, kG, kCos);
    FeedforwardEx FeedforwardController = new FeedforwardEx(feedforwardCoefficientsEx);

    DoubleSupplier positionSupplier = this::getAverageSlidesPosition;
    LowPassEstimator lowPassFilter = new LowPassEstimator(positionSupplier, a);

    BasicSystem basicSystem = new BasicSystem(lowPassFilter, PIDController, FeedforwardController);

    public WPILibMotionProfile motionProfile;
    WPILibMotionProfile.Constraints profileConstraints = new WPILibMotionProfile.Constraints(20, 20);
    public final ElapsedTime profileTimer = new ElapsedTime();

    public boolean isResettingLifts = false;

    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    int lastLeftPos;
    int lastRightPos;
    double lastVelo;

    public double releaseSpeedLimit = 0.25;

    public String leftSlideName = "leftSlide";
    public String rightSlideName = "rightSlide";

    public boolean isSpeeding = false;

    public boolean isReset = true;

    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(Gamepad gamepad) {
        isSpeeding = getAverageSlidesSpeed() > releaseSpeedLimit;
        isReset = getAverageSlidesPosition() == 0;
        lastVelo = getAverageSlidesVelocity();
    }

    public void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        setLastSlidesPosition();
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
        double power = basicSystem.update(getAverageLastSlidesPosition());
        setPower(power);
    }

    public void setSlidesTargetPosition(int targetPosition) {
        WPILibMotionProfile.State goal = new WPILibMotionProfile.State(targetPosition, 0);
        WPILibMotionProfile.State initial = new WPILibMotionProfile.State(getAverageSlidesPosition(), getAverageSlidesVelocity());
        motionProfile = new WPILibMotionProfile(profileConstraints, goal, initial);
        profileTimer.reset();
    }
    public void resetSlidesPosition() {
        isResettingLifts = true;
        setSlidesTargetPosition(0);
    }

    public boolean isProfileFinished() {
        if (motionProfile != null) {
            return motionProfile.isFinished(profileTimer.seconds());
        } else {
            return true;
        }
    }





    public int getAverageSlidesPosition() {
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition())/2;
    }

    public double getAverageSlidesVelocity() {
        return (leftSlide.getVelocity() + rightSlide.getVelocity())/2;
    }

    public double getAverageSlidesSpeed() {
        return (leftSlide.getPower() + rightSlide.getPower())/2;
    }

    public void setLastSlidesPosition() {
        lastLeftPos = leftSlide.getCurrentPosition();
        lastRightPos = rightSlide.getCurrentPosition();
    }

    public int getAverageLastSlidesPosition() {
        return (lastLeftPos + lastRightPos)/2;
    }
}