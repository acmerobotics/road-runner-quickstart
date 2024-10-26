package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideGroup {
    MotorGroup Slides;

    public SlideGroup(MotorGroup s){
        Slides = s;
    }

    public SlideGroup(HardwareMap hardwareMap){
        MotorEx SlideLeft = new MotorEx(hardwareMap, Constants.SlideLeftName);
        SlideLeft.setRunMode(Motor.RunMode.RawPower);
        SlideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MotorEx SlideRight = new MotorEx(hardwareMap, Constants.SlideRightName);
        SlideRight.setRunMode(Motor.RunMode.RawPower);
        SlideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        Slides = new MotorGroup(SlideLeft, SlideRight);
    }

    public double getSlidePosition() {
        return Slides.getCurrentPosition();
    }

    public double getSlidePositionInches() {
        return this.toInches(Slides.getCurrentPosition());
    }

    public void slidesUp(){
        Slides.set(-1);
    }

    public void slidesDown(){
        Slides.set(1);
    }

    public void slidesHoldPosition(){
        Slides.set(-0.009375);
    }

    private double toInches(double ticks){
        return ticks; // Calculate this
    }

    public void slidesToPosition(double inchesUp){
        double startPos = this.getSlidePositionInches();
        long startTime = System.currentTimeMillis();

        while(System.currentTimeMillis() - startTime < 10_000) {
            if (startPos > inchesUp) {
                this.slidesDown();
            } else if (startPos < inchesUp) {
                this.slidesUp();
            }

            if (this.getSlidePositionInches() - inchesUp < 0.5 | this.getSlidePositionInches() - inchesUp > -0.5) {
                this.slidesHoldPosition();
                break;
            }
        }
    }

    public void resetEncoder(){
        Slides.resetEncoder();
    }

}
