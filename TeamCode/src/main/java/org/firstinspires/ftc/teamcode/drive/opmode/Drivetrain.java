package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    //Hardware declarations - Im telling what motors are in the dt.
    private final DcMotor motorLeftBack;
    private final DcMotor motorLeftFront;
    private final DcMotor motorRightBack;
    private final DcMotor motorRightFront;

    public final static double LEFT_FRONT_MODIFIER = 1;
    public final static double LEFT_BACK_MODIFIER = -1;
    public final static double RIGHT_FRONT_MODIFIER = 1;
    public final static double RIGHT_BACK_MODIFIER = -1;

    //Constants - values that will remain the same

    //Constructor - makes all this into a class that we can make objects from
    public Drivetrain(DcMotor motorLeftBack, DcMotor motorLeftFront,
                      DcMotor motorRightBack, DcMotor motorRightFront) {
        this.motorLeftBack = motorLeftBack;
        this.motorLeftFront = motorLeftFront;
        this.motorRightBack = motorRightBack;
        this.motorRightFront = motorRightFront;
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    //each of the different directions
    enum Direction {
        FORWARD,
        FORWARD_LEFT,
        FORWARD_RIGHT,
        LEFT,
        RIGHT,
        BACKWARD,
        BACKWARD_LEFT,
        BACKWARD_RIGHT,
        LEFT_TURN,
        RIGHT_TURN,
    }

    //Methods - activities the dt will do

    //Below has all the directions under a switch case.
    public void driveDirection(final Direction direction, final double power) {
        final double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;
        switch (direction) {
            case FORWARD:
                leftFrontPower = leftBackPower = rightFrontPower = rightBackPower = power;
                break;
            case FORWARD_LEFT:
                rightFrontPower = leftBackPower = power;
                leftFrontPower = rightBackPower = 0;
                break;
            case FORWARD_RIGHT:
                rightBackPower = leftFrontPower = power;
                rightFrontPower = leftBackPower = 0;
                break;
            case LEFT:
                rightFrontPower = leftBackPower = power;
                rightBackPower = leftFrontPower = -power;
                break;
            case BACKWARD_LEFT:
                leftBackPower = rightBackPower = -power;
                leftFrontPower = rightFrontPower = 0;
                break;
            case BACKWARD:
                leftFrontPower = leftBackPower = rightFrontPower = rightBackPower = -power;
                break;
            case BACKWARD_RIGHT:
                rightBackPower = leftBackPower = power;
                rightFrontPower = leftFrontPower = 0;
                break;
            case RIGHT:
                rightFrontPower = leftBackPower = -power;
                rightBackPower = leftFrontPower = power;
                break;
            case LEFT_TURN:
                rightBackPower = rightFrontPower = power;
                leftBackPower = leftFrontPower = -power;
                break;
            case RIGHT_TURN:
                rightBackPower = rightFrontPower = -power;
                leftBackPower = leftFrontPower = power;
                break;
            default:
                leftFrontPower = leftBackPower = rightFrontPower = rightBackPower = 0;
        }

        motorLeftFront.setPower(LEFT_FRONT_MODIFIER * leftFrontPower);
        motorLeftBack.setPower(LEFT_BACK_MODIFIER * leftBackPower);
        motorRightFront.setPower(RIGHT_FRONT_MODIFIER * rightFrontPower);
        motorRightBack.setPower(RIGHT_BACK_MODIFIER * rightBackPower);
    }

    public void driveRobotCentricFromJoysticks(final double ly, final double lx, final double rx) {
        final double denominator = Math.max(Math.abs(-ly) + Math.abs(lx) + Math.abs(rx), 1);
        final double leftFrontPower = (-ly + lx + rx) / denominator;
        final double leftBackPower = (-ly - lx + rx) / denominator;
        final double rightFrontPower = (-ly - lx - rx) / denominator;
        final double rightBackPower = (-ly + lx - rx) / denominator;

        motorLeftFront.setPower(leftFrontPower);
        motorLeftBack.setPower(leftBackPower);
        motorRightFront.setPower(rightFrontPower);
        motorRightBack.setPower(rightBackPower);
    }
}
