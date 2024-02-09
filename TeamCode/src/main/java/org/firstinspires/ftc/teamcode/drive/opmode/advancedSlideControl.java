package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class advancedSlideControl {
    private final HardwareMap hardwareMap;

    // Create motor, servo, and gyro objects
    private final DcMotorEx slideLeft;
    private final DcMotorEx slideRight;
    private final DcMotorEx slideTop;
    private final Servo leftGripServo;
    private final Servo rightGripServo;

    // Create robot class
    public advancedSlideControl(final HardwareMap _hardwareMap) {
        // Pass variables through to Robot class
        hardwareMap = _hardwareMap;

        // Do the same thing we did earlier with the drive motors, just for the slide
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideleft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideTop = hardwareMap.get(DcMotorEx.class, "slideTop");

        leftGripServo = hardwareMap.servo.get("leftGripServo");
        rightGripServo = hardwareMap.servo.get("rightGripServo");
    }

    // This function accepts a motor mode followed by a list of DcMotor objects
    private void setMotorMode(DcMotorEx.RunMode mode, DcMotorEx... motors) {
        // Iterate over each DcMotor object and set their motor mode
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    public void stopAndResetMotors() {
        setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER, slideLeft, slideRight, slideTop);
    }

    public void restartMotors() {
        setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION, slideLeft, slideRight, slideTop);
    }

    public void setHeight(int height) {
        slideLeft.setTargetPosition(height);
        slideRight.setTargetPosition(-height);
    }

    public void setExtension(int ext) {
        slideTop.setTargetPosition(-ext);
    }

    public void setSlideVelocity(int vel, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(vel);
        }
    }

    public void setGrip(boolean grip, Servo leftServo, Servo rightServo) {
        double gripPower = grip ? 0 : 1;
        double leftPos = ((-1 * gripPower + 1) / 3) + 5.0/9;
        double rightPos = (gripPower / 3) + 2.0/3;

        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);
    }
}