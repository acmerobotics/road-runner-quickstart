package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.NoFeedback;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Arm extends Subsystem {

    protected DcMotorEx slideLeft;
    protected DcMotorEx slideRight;

    protected double slideSetpoint = SlidePositions.down;

    protected FeedbackController slideControllerLeft = new NoFeedback();
    protected FeedbackController slideControllerRight = new NoFeedback();

    public static class SlidePositions {
        public static final double down = 0;
        public static final double low = 1;
        public static final double high = 2;
    }

    protected Servo wrist;
    protected Servo left_arm;
    protected Servo right_arm;

    private void commonInit(HardwareMap hwMap) {
        slideLeft = hwMap.get(DcMotorEx.class, "left_lift");
        slideRight = hwMap.get(DcMotorEx.class, "right_lift");

        left_arm = hwMap.get(Servo.class, "arm_left");
        right_arm = hwMap.get(Servo.class, "arm_right");

        wrist = hwMap.get(Servo.class, "wrist");
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);

        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
    }

    @Override
    public void periodic() {
        // check this method for getting the position lol it's wrong
//        double leftPower = slideControllerLeft.calculate(slideSetpoint, slideLeft.getCurrentPosition());
//        double rightPower = slideControllerRight.calculate(slideSetpoint, slideRight.getCurrentPosition());
//
//        slideLeft.setPower(leftPower);
//        slideRight.setPower(rightPower);

//        wrist.setPosition(0);

        setArmPosition(0.1);

    }

    @Override
    public void shutdown() {
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    public static double encoderTicksToInches(double ticks) {
        double SPOOL_SIZE_IN = 1.270 / 2.0;
        double GEAR_RATIO = 13.7;
        double TICKS_PER_REV = 28;
        return SPOOL_SIZE_IN * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    protected void setArmPosition(double position) {
        left_arm.setPosition(1 - position);
        right_arm.setPosition(position);
    }

}