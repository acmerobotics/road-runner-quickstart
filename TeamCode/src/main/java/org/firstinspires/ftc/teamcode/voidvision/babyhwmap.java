package org.firstinspires.ftc.teamcode.voidvision;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class babyhwmap extends HardwareMapUtil {
    HardwareMap hwmap = null;

    public DcMotor leftfrontDrive = null;

    public DcMotor rightfrontDrive = null;

    public DcMotor leftbackDrive = null;

    public DcMotor rightbackDrive = null;

    public DcMotor armMotorOne = null;
    public DcMotor armMotorTwo = null;
    public CRServo armServo = null;
    public Servo posServo = null;
    public Servo servo = null;
//

    public void init(HardwareMap ahwMap){
        hwMap=ahwMap;
        leftfrontDrive = HardwareInitMotor("leftFront", true);
        rightfrontDrive = HardwareInitMotor("rightFront", false);
        leftbackDrive = HardwareInitMotor("leftBack", true);
        rightbackDrive = HardwareInitMotor("rightBack", false);
        //servo = HardwareInitServo("servo",0);
        //armMotorOne = HardwareInitMotor("arm_1", true);
        //armMotorTwo = HardwareInitMotor("arm_2", true);
        //armServo = hwMap.get(CRServo.class, "servo");
        //posServo = hwMap.get(Servo.class, "posServo");
        //servo = hwmap.get(Servo.class, "servo");


        //armServo.setDirection(DcMotorSimple.Direction.FORWARD);

        leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}