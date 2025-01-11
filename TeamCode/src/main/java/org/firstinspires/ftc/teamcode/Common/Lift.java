package org.firstinspires.ftc.teamcode.Common;

import static org.firstinspires.ftc.teamcode.Common.Constants.TICKS_TO_INCHES_LIFT;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    MotorGroup Slides;
    MotorEx SlideLeft;
    MotorEx SlideRight;

    public Lift(MotorGroup s){
        Slides = s;
    }

    public Lift(HardwareMap hardwareMap){
        MotorEx SL = new MotorEx(hardwareMap, Constants.SlideLeftName);
        SL.setRunMode(Motor.RunMode.RawPower);
        SL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.SlideLeft = SL;

        MotorEx SR = new MotorEx(hardwareMap, Constants.SlideRightName);
        SR.setRunMode(Motor.RunMode.RawPower);
        SR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.SlideRight = SR;

        Slides = new MotorGroup(SlideLeft, SlideRight);
    }

    public double getSlidePosition() {
        return Slides.getCurrentPosition();
    }

    public double getSlidePositionInches() {
        return toInches(Slides.getCurrentPosition());
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

    private static double toTicks(double inches){
        return inches / TICKS_TO_INCHES_LIFT; // Calculate this
    }

    private static double toInches(double ticks){
        return ticks * TICKS_TO_INCHES_LIFT; // Calculate this
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


    public Action SlidesToBar() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    SlideLeft.set(0.8);
                    SlideRight.set(-0.8);
                    initialized = true;
                }

                double pos = SlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos == toTicks(13))  {
                    SlideLeft.set(0.1);

                    SlideRight.set(-0.1);
                    return true;
                } else {
                    SlideLeft.set(0.8);
                    SlideRight.set(-0.8);
                    return false;
                }
            }
        };
    }
    public Action SlidesToBar_new() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    SlideLeft.set(0.8);
                    SlideRight.set(-0.8);
                    initialized = true;
                }

                double pos = SlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos >= toTicks(14)) {
                    SlideLeft.set(0.15);
                    SlideRight.set(-0.15);
                    return false;
                }
                return true;
            }
        };
    }




    public Action FirstBar() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                SlideLeft.set(0.8);
                SlideRight.set(-0.8);
                double pos = SlideLeft.getCurrentPosition();
               if (pos>=toInches(8)) {
                   SlideLeft.set(0);
                   SlideRight.set(0);
                }
                return false;
            }
        };
    }



    public Action SlidesToNet() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    SlideLeft.set(0.8);
                    SlideRight.set(-0.8);
                    initialized = true;
                }

                double pos = SlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < toTicks(47)) {
                    return true;
                } else {
                    SlideLeft.set(0.15);
                    SlideRight.set(-0.15);
                    return false;
                }
            }
        };
    }

    public Action SlidesDown() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    SlideLeft.set(0.8);
                    SlideRight.set(-0.8);
                    initialized = true;
                }

                double pos = SlideLeft.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > toTicks(1)) {
                    return true;
                } else {
                    SlideLeft.set(0.15);
                    SlideRight.set(-0.2);
                    return false;
                }
            }
        };
    }
}
