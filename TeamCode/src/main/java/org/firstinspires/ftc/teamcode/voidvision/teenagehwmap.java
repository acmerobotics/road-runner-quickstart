package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * teenagehwmap class defines the hardware mapping for the robot's motors, servos, and sensors.
 * It initializes and configures the drive motors, arm mechanisms, and sensor hardware.
 */
public class teenagehwmap extends HardwareMapUtil {

    // Declare hardware components
    HardwareMap hwmap = null;
    public DcMotor leftfrontDrive = null;
    public DcMotor rightfrontDrive = null;
    public DcMotor leftbackDrive = null;
    public DcMotor rightbackDrive = null;
    public DcMotor liftMotor = null;
    public DcMotor armMotorTwo = null;
    public CRServo armServo = null;
    public Servo posServo = null;
    public Servo range1Servo = null;  // Servo for left range
    public Servo range2Servo = null;  // Servo for right range
    public Servo basketServo1 = null; // Servo for left basket
    public Servo basketServo2 = null; // Servo for right basket
    public CRServo intakeServo = null;
    public ColorSensor colorSensor = null;

    // Default positions for range and basket servos
    public double Finalrange = 0.45;
    //.4 is good
    //.6 is bad
    public double FinalrangeBasket = 0.5;

    /**
     * Initializes all hardware components and sets their initial states.
     *
     * @param ahwMap The HardwareMap from the FTC SDK that maps the hardware configuration.
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Initialize drive motors
        leftfrontDrive = HardwareInitMotor("leftFront", true);
        rightfrontDrive = HardwareInitMotor("rightFront", false);
        leftbackDrive = HardwareInitMotor("leftBack", true);
        rightbackDrive = HardwareInitMotor("rightBack", false);

        // Initialize arm motors and servos (commented out if not needed yet)
        liftMotor = HardwareInitMotor("liftMotor", false);
        // armMotorTwo = HardwareInitMotor("arm_2", true);
        // armServo = hwMap.get(CRServo.class, "servo");
        // posServo = hwMap.get(Servo.class, "posServo");

        // Initialize range and basket servos
        range1Servo = HardwareInitServo("hippo1", 0); // Left range servo
        range2Servo = HardwareInitServo("hippo2", Finalrange); // Right range servo
        intakeServo = HardwareInitCRServo("intake", true); // Intake servo
        basketServo1 = HardwareInitServo("basket1", 0); // Left basket servo
        basketServo2 = HardwareInitServo("basket2", FinalrangeBasket); // Right basket servo

        // Initialize color sensor (commented out if not needed yet)
        // colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        /** Set servo directions */
        // armServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        /** Set motor zero power behavior (motors stop when zero power is applied) */
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
