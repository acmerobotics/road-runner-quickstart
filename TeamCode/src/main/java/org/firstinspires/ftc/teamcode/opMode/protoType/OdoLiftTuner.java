package org.firstinspires.ftc.teamcode.opMode.protoType;

//Code for playing around with servos. Go on, experiment!
//Instructions for doing this in odolift mech

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoTest", group = "prototype")
public class OdoLiftTuner extends LinearOpMode {
    private Servo testServoLeft;
    private Servo testServoRight;
    private Servo testServoFront;

    public static double START_LEFT = 0.0;
    public static double END_LEFT = 1.0;

    public static double START_RIGHT = 0.0;
    public static double END_RIGHT = 0.0;

    public static double START_FRONT = 0.0;
    public static double END_FRONT = 0.0;
    public void init(HardwareMap hwMap){

        testServoLeft = hwMap.servo.get("deposit");


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

                    testServoLeft.setPosition(START_LEFT);


                }
            }

            if(gamepad1.b){
                formerB = true;
            }
            if(formerB){
                if(!gamepad1.b){
                    formerB = false;

                    testServoLeft.setPosition(END_LEFT);

                }
            }
        }
    }
}