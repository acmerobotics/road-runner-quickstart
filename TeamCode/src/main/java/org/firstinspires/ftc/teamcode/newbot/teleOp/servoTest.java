package org.firstinspires.ftc.teamcode.newbot.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp
public class servoTest extends LinearOpMode {
    public Servo baseLeft = null;
    public Servo baseRight = null;
    public Servo elbow = null;


    public void runOpMode() {
        baseRight = hardwareMap.get(Servo.class, "baseRight");
        baseLeft = hardwareMap.get(Servo.class, "baseLeft");
        elbow = hardwareMap.get(Servo.class, "elbow");
        //baseRight.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.b) {
                baseRight.setPosition(0);
                baseLeft.setPosition(0);
            }
            //open
            if (gamepad1.x) {
                baseRight.setPosition(0.2);
                baseLeft.setPosition(0.48);
            }
        }



    }
}



