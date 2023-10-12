package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanism;
public class Claw extends Mechanism {

        Servo rotator;
        Servo leftProng;
        Servo rightProng;

        public String rotatorName = "rotator";
        public String leftProngName = "leftProng";
        public String rightProngName = "rightProng";

        public double tiltedRight = 0.5;
        public double tiltedLeft = -0.5;
        public double straight = 0;
        public double releasePosition = 0;
        public double clampPosition = 1;

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
            targetTilt = straight;
        }

        @Override
        public void loop(Gamepad gamepad) {
            switch (activeTiltState) {
                case LEFT:
                    targetTilt = tiltedLeft;
                    tilt(tiltedLeft);
                    break;
                case RIGHT:
                    targetTilt = tiltedRight;
                    tilt(tiltedRight);
                    break;
                case CENTER:
                    targetTilt = straight;
                    tilt(straight);
                    break;
            }

            isRotatorInPosition = rotator.getPosition() == targetTilt;

            isLeftClamped = leftProng.getPosition() == clampPosition;
            isRightClamped = rightProng.getPosition() == clampPosition;
            isLeftReleased = leftProng.getPosition() == releasePosition;
            isRightReleased = rightProng.getPosition() == releasePosition;
        }

        void tilt(double position) {
            rotator.setPosition(position);
        }

        public void clampServo(Servo servo) {
            servo.setPosition(clampPosition);
        }

        public void releaseServo(Servo servo) {
            servo.setPosition(releasePosition);
        }

        public void setActiveTiltState(TiltState tiltState) {
            activeTiltState = tiltState;
        }
    }


