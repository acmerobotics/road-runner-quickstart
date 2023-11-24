package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Slide {

    private final HardwareMap hardwareMap;

    private final Telemetry telemetry;

    private final DcMotorEx slideLeft, slideRight, slideTop;

    private final Servo leftGripServo, rightGripServo;

    private boolean inProgress = false;

    private final int maxVel = 2800;
    private final int maxHeight = 4400, minHeight = 0;
    private final int maxExt = 2200, minExt = 0;

    private int targetPos;

    public enum heights {
        LOW,
        MID,
        HIGH
    }

    public Slide(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideTop = hardwareMap.get(DcMotorEx.class, "slideTop");

        leftGripServo = hardwareMap.servo.get("leftGripServo");
        rightGripServo = hardwareMap.servo.get("rightGripServo");

        slideLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void setMotorMode(DcMotorEx.RunMode mode, DcMotorEx... motors) {
        // Iterate over each DcMotor object and set their motor mode
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }

    // Call setMotorMode() to turn off and reset the encoders on all slide motors
    public void stopAndResetMotors() {
        setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER, slideLeft, slideRight, slideTop);
    }

    // Call setMotorMode() to turn on all slide motors
    public void restartMotors() {
        setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION, slideLeft, slideRight, slideTop);
    }

    private void setBrakeMode(DcMotor.ZeroPowerBehavior mode, DcMotorEx... motors) {
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(mode);
        }
    }

    public void runSlidesWithBrakes() {
        setBrakeMode(DcMotor.ZeroPowerBehavior.BRAKE, slideLeft, slideRight, slideTop);
    }

    // Bundles all the functions needed to initialize the arm controls
    public void initArm() {
        stopAndResetMotors();
        setGrip(false);
        setSlideVelocity(0, slideLeft, slideRight, slideTop);
        setHeight(0);
        setExtension(0);
        restartMotors();
    }

    public void setHeight(int height) {
        slideLeft.setTargetPosition(height);
        slideRight.setTargetPosition(-height);
        telemetry.addData("leftpos", slideLeft.getCurrentPosition());
        telemetry.addData("lefttarg", slideLeft.getTargetPosition());
        telemetry.addData("rightpos", slideRight.getCurrentPosition());
        telemetry.addData("righttart", slideRight.getTargetPosition());
    }

    // Set the target encoders position of the horizontal slide
    public void setExtension(int ext) {
        slideTop.setTargetPosition(ext);
    }

    // Iterate over a list of motors and set them to a provided velocity in ticks/second
    public void setSlideVelocity(double vel, DcMotorEx... motors) {
        for (DcMotorEx motor : motors) {
            motor.setVelocity(vel);
        }
    }

    // Takes a boolean grip value and does the math to convert it to a servo position
    public void setGrip(boolean grip) {
        double leftOpen = 0.0, leftClosed = 105.0;
        double rightOpen = 270.0, rightClosed = 175.0;

        if (grip) {
            leftGripServo.setPosition(leftClosed / 270);
            rightGripServo.setPosition(rightClosed / 270);
        } else if (!grip) {
            leftGripServo.setPosition(leftOpen / 270);
            rightGripServo.setPosition(rightOpen / 270);
        }
    }

    public void manualHeightControl(double velScale) {
        // TODO: see if setting int.max and int.min is better
        if (velScale > 0.0) {
            setHeight(Integer.MAX_VALUE);
            setSlideVelocity(maxVel * velScale, slideLeft, slideRight);
            inProgress = false;
        } else if (velScale < 0.0) {
            setHeight(Integer.MIN_VALUE);
            setSlideVelocity(maxVel * velScale, slideLeft, slideRight);
            inProgress = false;
        } else if (!inProgress) {
            setSlideVelocity(0, slideLeft, slideRight);
        }
    }

    public void manualExtensionControl(double velScale) {
        if (velScale > 0.1) {
            setExtension(Integer.MAX_VALUE);
            setSlideVelocity(maxVel * velScale, slideTop);
        } else if (velScale < -0.1) {
            setExtension(Integer.MIN_VALUE);
            setSlideVelocity(maxVel * velScale, slideTop);
        } else {
            setSlideVelocity(0, slideTop);
        }
    }

    public void goToJunction(heights junction) {
        switch (junction) {
            case LOW:
                setHeight(2040);
                break;
            case MID:
                setHeight(3200);
                break;
            case HIGH:
                setHeight(4200);
                break;
        }
        setSlideVelocity(-1 * maxVel, slideLeft, slideRight);
        inProgress = true;
    }
}

