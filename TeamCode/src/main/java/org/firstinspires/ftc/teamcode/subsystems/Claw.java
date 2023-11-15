package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;
public class Claw extends Mechanism {

        public Servo rotator;
        public Servo leftProng;
        public Servo rightProng;

        public String rotatorName = "rotator";
        public String leftProngName = "leftProng";
        public String rightProngName = "rightProng";

        public double tiltedRightPos = 0.77;
        public double tiltedLeftPos = 0.27;
        public double straight = 0.52;
        public double releasePosition = 0.1;
        public double leftReleasePosition = 0.2;
        public double clampPosition = 0.015;

        public boolean isLeftClamped = false;
        public boolean isLeftReleased = true;
        public boolean isRightClamped = false;
        public boolean isRightReleased = true;

        public boolean isRotatorInPosition = false;

        public enum TiltState {
            LEFT, RIGHT, CENTER
        }

        TiltState activeTiltState = TiltState.CENTER;

        public double targetTilt = 0;

        @Override
        public void init(HardwareMap hwMap) {
            rotator = hwMap.get(Servo.class, rotatorName);
            leftProng = hwMap.get(Servo.class, leftProngName);
            rightProng = hwMap.get(Servo.class, rightProngName);
            leftProng.setDirection(Servo.Direction.REVERSE);
            targetTilt = straight;
        }

        @Override
        public void loop(Gamepad gamepad) {
            switch (activeTiltState) {
                case LEFT:
                    targetTilt = tiltedLeftPos;
                    tilt(tiltedLeftPos);
                    break;
                case RIGHT:
                    targetTilt = tiltedRightPos;
                    tilt(tiltedRightPos);
                    break;
                case CENTER:
                    targetTilt = straight;
                    tilt(straight);
                    break;
            }

            isRotatorInPosition = rotator.getPosition() == targetTilt;

            isLeftClamped = Math.abs(leftProng.getPosition() - clampPosition) < 0.01;
            isRightClamped = Math.abs(rightProng.getPosition() - clampPosition) < 0.01;
            isLeftReleased = Math.abs(leftProng.getPosition() - leftReleasePosition) < 0.01;
            isRightReleased = Math.abs(rightProng.getPosition() - releasePosition) < 0.01;
        }

        @Override
        public void telemetry(Telemetry telemetry) {
            telemetry.addData("Rotator Position", rotator.getPosition());
            telemetry.addData("Left Prong Position", leftProng.getPosition());
            telemetry.addData("Right Prong Position", rightProng.getPosition());
            telemetry.addData("Target Tilt", targetTilt);
            telemetry.addData("Active Tilt State", activeTiltState);
            telemetry.addData("isRotatorInPosition", isRotatorInPosition);
            telemetry.addData("isLeftClamped", isLeftClamped);
            telemetry.addData("isRightClamped", isRightClamped);
            telemetry.addData("isLeftReleased", isLeftReleased);
            telemetry.addData("isRightReleased", isRightReleased);
        }

        public void tilt(double position) {
            rotator.setPosition(position);
        }

        public void clampServo(Servo servo) {
            servo.setPosition(clampPosition);
        }

        public void releaseServo(Servo servo) {
            servo.setPosition(releasePosition);
        }

        public void releaseLeftServo(Servo servo) {
            servo.setPosition(leftReleasePosition);
        }

        public void setActiveTiltState(TiltState tiltState) {
            activeTiltState = tiltState;
        }
    }


