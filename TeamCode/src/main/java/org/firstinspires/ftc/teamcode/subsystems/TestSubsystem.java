
// This subsystem is used for testing motors and general concepts
// It should not be in functional code because it will provide no value to the robot


package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.TEST_MOTOR_MAX_POWER;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestSubsystem extends SubsystemBase {

    // Defining the test motor
    private final DcMotor testDcMotor;

    /**
     * Simple subsystem for testing motors - Originally for proof of concept of FTCLib
     * @param testMotor Motor to test
     */
    public TestSubsystem(DcMotor testMotor) {
        // This gets the information for the motor
        this.testDcMotor = testMotor;
    }

    // Simple methods to allow for control over the motor with constants
    public void forward() {
        testDcMotor.setPower(TEST_MOTOR_MAX_POWER);
    }

    public void backward() {
        testDcMotor.setPower(-TEST_MOTOR_MAX_POWER);
    }

    public void stop() {testDcMotor.setPower(0);}

    @Override
    public void periodic() {
    }

}

