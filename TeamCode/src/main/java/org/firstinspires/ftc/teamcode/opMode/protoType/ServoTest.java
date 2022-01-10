package org.firstinspires.ftc.teamcode.opMode.protoType;

//Code for playing around with servos. Go on, experiment!
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoTest",group = "Teleop")
public class ServoTest extends LinearOpMode {
    private Servo testServoMain;
    private Servo testServoSupp;
    public static double START = 0.1;
    public static double END = 0.6;
    public static double START2 = 0.98;
    public static double END2 = 0.5;
            ;
    //Servo Main: START: 0.45; END: 0.95
    //Servo Supp: START: 0.6; END: 0.1
    //Clamp Servo: START: 0.1; END: 0.6
    public void init(HardwareMap hwMap){

        testServoMain = hwMap.servo.get("armServoL");
        testServoSupp = hwMap.servo.get("armServoR");
    }
    @Override
    public void runOpMode() throws InterruptedException{
        boolean formerA = false;
        boolean formerB = false;
        init(hardwareMap);
        waitForStart();
        while(opModeIsActive()&& !isStopRequested()){
            if(gamepad1.a){
                formerA = true;
            }
            if(formerA){
                if(!gamepad1.a){
                    formerA = false;
                    testServoMain.setPosition(START);
                    testServoSupp.setPosition(START2);
                }
            }

            if(gamepad1.b){
                formerB = true;
            }
            if(formerB){
                if(!gamepad1.b){
                    formerB = false;
                    testServoMain.setPosition(END);
                    testServoSupp.setPosition(END2);
                }
            }
        }
    }
}