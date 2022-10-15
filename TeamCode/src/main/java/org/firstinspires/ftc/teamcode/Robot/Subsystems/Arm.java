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
        public static double down = 0;
        public static double low = 1;
        public static double high = 2;
    }

    protected Servo wrist;

    private void commonInit(HardwareMap hwMap) {
//        slideLeft = hwMap.get(DcMotorEx.class, "slide_left");
//        slideRight = hwMap.get(DcMotorEx.class, "slide_right");

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
    }

    @Override
    public void shutdown() {
        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }
}
