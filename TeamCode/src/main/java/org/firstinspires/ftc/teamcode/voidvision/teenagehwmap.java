package org.firstinspires.ftc.teamcode.voidvision;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class teenagehwmap extends HardwareMapUtil {
    HardwareMap hwmap = null;

    public DcMotor leftfrontDrive = null;

    public DcMotor rightfrontDrive = null;

    public DcMotor leftbackDrive = null;

    public DcMotor rightbackDrive = null;

    public DcMotor armMotorOne = null;
    public DcMotor armMotorTwo = null;
    public CRServo armServo = null;
    public Servo posServo = null;
    public Servo range1Servo = null;
    public Servo range2Servo = null;
    public Servo basketServo1 = null;
    public Servo basketServo2 = null;
    public CRServo intakeServo = null;
    public ColorSensor colorSensor = null;


//

    public void init(HardwareMap ahwMap){
        hwMap=ahwMap;
        leftfrontDrive = HardwareInitMotor("leftFront", true);
        rightfrontDrive = HardwareInitMotor("rightFront", false);
        leftbackDrive = HardwareInitMotor("leftBack", true);
        rightbackDrive = HardwareInitMotor("rightBack", false);
        //armMotorOne = HardwareInitMotor("arm_1", true);
        //armMotorTwo = HardwareInitMotor("arm_2", true);
        //armServo = hwMap.get(CRServo.class, "servo");
        //posServo = hwMap.get(Servo.class, "posServo");
        range1Servo = HardwareInitServo("hippo1",0);
        range2Servo = HardwareInitServo("hippo2",0);
        intakeServo = HardwareInitCRServo("intake",true);
        //range1Servo = hwMap.get(Servo.class, "range1Servo");
        //range2Servo = hwMap.get(Servo.class, "range2Servo");
        //intakeServo = hwMap.get(CRServo.class, "intakeServo");

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        /**Set Servos**/
        //armServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);



        /**Set Motors**/
        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}