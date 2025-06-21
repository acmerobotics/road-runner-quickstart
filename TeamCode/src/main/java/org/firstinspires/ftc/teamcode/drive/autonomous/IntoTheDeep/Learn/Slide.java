package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Slide {

    public final DcMotor leftSlide, rightSlide;
    public Telemetry telemetry;

    public Slide(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class MoveToPosition implements Action {
        private final int targetPosition;
        private final double power;
        private boolean initialized = false;

        public MoveToPosition(int targetPosition, double power) {
            this.targetPosition = targetPosition;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setTargetPosition(targetPosition);
                rightSlide.setTargetPosition(targetPosition);

                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftSlide.setPower(power);
                rightSlide.setPower(power);

                initialized = true;
            }

            telemetry.addData("Left Slide Position", leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide Position", rightSlide.getCurrentPosition());
            telemetry.update();


            // Check if both motors are still running towards their target positions
            if (Math.abs(leftSlide.getCurrentPosition() - targetPosition) > 5 || Math.abs(rightSlide.getCurrentPosition() - targetPosition) > 5) {
                return true; // Still moving
            }

            // Once the slide has reached the target, stop the motors
            leftSlide.setPower(0);
            rightSlide.setPower(0);
            return false; // Action complete
        }
    }


    public class MoveToHome implements Action {
        private final double power;
        private final long timeThreshold = 500;
        private final double stabilityThreshold = 5;
        private double lastLeftPos, lastRightPos;
        private long stableStartTime = 0;

        public MoveToHome(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double currentLeftPos = leftSlide.getCurrentPosition();
            double currentRightPos = rightSlide.getCurrentPosition();

            // Set the motors to move down
            leftSlide.setPower(-power);
            rightSlide.setPower(-power);

            // Check for stability using a movement threshold
            boolean isStable = (Math.abs(currentLeftPos - lastLeftPos) < stabilityThreshold &&
                    Math.abs(currentRightPos - lastRightPos) < stabilityThreshold);

            if (isStable) {
                if (stableStartTime == 0) {
                    stableStartTime = System.currentTimeMillis();
                }
            } else {
                stableStartTime = 0;
            }

            lastLeftPos = currentLeftPos;
            lastRightPos = currentRightPos;

            // Telemetry for debugging
            telemetry.addData("Left Slide Position", currentLeftPos);
            telemetry.addData("Right Slide Position", currentRightPos);
            telemetry.addData("Stable Time Counter", stableStartTime == 0 ? 0 : System.currentTimeMillis() - stableStartTime);
            telemetry.update();

            // Stop motors and reset encoders after stability is confirmed
            if (stableStartTime != 0 && System.currentTimeMillis() - stableStartTime >= timeThreshold) {
                leftSlide.setPower(0);
                rightSlide.setPower(0);

                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                return true; // Action complete
            }

            return false; // Action still in progress
        }
    }
        public Action moveToHome(double power) {
            return new MoveToHome(power);}

        public Action moveToPosition(int targetPosition, double power) {
            return new MoveToPosition(targetPosition, power);}

}
