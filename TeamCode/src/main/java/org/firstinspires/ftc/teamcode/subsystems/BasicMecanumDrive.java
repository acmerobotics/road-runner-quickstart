package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BasicMecanumDrive extends SubsystemBase {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    /**
     * Example of a basic mecanum drive system.
     *
     * @param frontLeft  Front Left Motor
     * @param frontRight Front Right Motor
     * @param backLeft   Back Left Motor
     * @param backRight  Back Right Motor
     */
    public BasicMecanumDrive(
            DcMotor frontLeft, DcMotor frontRight,
            DcMotor backLeft, DcMotor backRight) {

        // Set the motor objects
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        //Initialize direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void periodic() {

    }

    public void Drive(Translation2d position) {
        double translationX = position.getX();
        double translationY = position.getY();
        double currentX = 0; // TODO Get the position values from IMU or Odometry
        double currentY = 0;
        double angle = Math.sinh((translationX - currentX)/(translationY - currentY));
        double pi = 3.14159265;

        // Setting the powers
        double x = Math.sin((pi * angle) / 180);
        double y = Math.cos((pi * angle) / 180);

        frontLeft.setPower(y + x);
        frontRight.setPower(y - x);
        backLeft.setPower(y - x);
        backRight.setPower(y + x);

        // TODO Find out how to get to position

    }
}