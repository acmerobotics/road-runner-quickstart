package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.NoFeedback;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Arm extends Subsystem {


    protected DcMotorEx slideLeft;
    protected DcMotorEx slideRight;

    protected double slideSetpoint = SlidePositions.down;

    protected FeedbackController slideControllerLeft = new NoFeedback();
    protected FeedbackController slideControllerRight = new NoFeedback();

    public static class SlidePositions {
        public static double down = 0;
        public static double low = 1;
        public static double high = 2;
    }

    private void commonInit(HardwareMap hwMap) {
        slideLeft = hwMap.get(DcMotorEx.class, "slide_left");
        slideRight = hwMap.get(DcMotorEx.class, "slide_right");
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);

        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
    }

    @Override
    public void periodic() {
        // check this method for getting the position lol it's wrong
        double leftPower = slideControllerLeft.calculate(slideSetpoint,
                encoderTicksToInches(slideLeft.getCurrentPosition()));

        double rightPower = slideControllerRight.calculate(slideSetpoint,
                encoderTicksToInches(slideRight.getCurrentPosition()));

        slideLeft.setPower(leftPower);
        slideRight.setPower(rightPower);
    }

    @Override
    public void shutdown() {
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public static double encoderTicksToInches(double ticks) {
        double SPOOL_SIZE_IN = 1.270 / 2.0;
        double GEAR_RATIO = 13.7;
        double TICKS_PER_REV = 28;
        return SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
