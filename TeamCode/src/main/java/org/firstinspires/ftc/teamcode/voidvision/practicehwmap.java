package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class initializes and maps the robot's hardware for practice operations.
 * It includes drive motors, a linear actuator, and a continuous rotation servo (CRServo).
 */
public class practicehwmap extends HardwareMapUtil {

    // Reference to the robot's hardware map
    HardwareMap hwmap = null;

    // Drive motor variables
    public DcMotor leftfrontDrive = null;
    public DcMotor rightfrontDrive = null;
    public DcMotor leftbackDrive = null;
    public DcMotor rightbackDrive = null;

    // Linear actuator motor
    public DcMotor linearActuator = null;

    // Continuous rotation servo
    public CRServo theServo = null;

    /**
     * Initializes the hardware components of the robot.
     * @param ahwMap Hardware map passed from the OpMode.
     */
    public void init(HardwareMap ahwMap) {
        // Assign the hardware map reference
        hwMap = ahwMap;

        // Initialize the motors with hardware mapping and direction
        leftfrontDrive = HardwareInitMotor("lfD", true);
        rightfrontDrive = HardwareInitMotor("rfD", false);
        leftbackDrive = HardwareInitMotor("lbD", true);
        rightbackDrive = HardwareInitMotor("rbD", false);
        linearActuator = HardwareInitMotor("lA", true);

        // Initialize the continuous rotation servo
        theServo = hwMap.get(CRServo.class, "the");

        // Set all motors to brake when power is set to zero
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
