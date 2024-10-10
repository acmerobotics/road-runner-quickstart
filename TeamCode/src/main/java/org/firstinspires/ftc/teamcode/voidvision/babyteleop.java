package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name="baby teleop", group="Pushbot")
public class babyteleop extends LinearOpMode {
    babyhwmap robot=new babyhwmap();
//
    private ElapsedTime runtime = new ElapsedTime();

    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount ;
    static double open = .3;
    static double closed = .5;
    static double direction = -1;


    private boolean changed1 = false;

    public void runOpMode(){
        robot.init(hardwareMap);

        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            //Drive

            fwdBackPower = direction * -gamepad1.left_stick_y * slowamount;
            strafePower = direction * -gamepad1.left_stick_x * slowamount;
            turnPower = gamepad1.right_stick_x * slowamount;

            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);


            robot.leftfrontDrive.setPower(lfPower);
            robot.leftbackDrive.setPower(lbPower);
            robot.rightfrontDrive.setPower(rfPower);
            robot.rightbackDrive.setPower(rbPower);

            slowamount = .5;


            if (gamepad1.right_bumper){
                flipWheelConfigurationBackward();
                telemetry.addData("Direction-Backward:", direction);

            }
            else{
                flipWheelConfigurationNormal();
                telemetry.addData("Direction-Normal:", direction);

            }


            if(gamepad1.left_bumper){
                slowamount = .25;
            }
            else{slowamount = gamepad1.left_trigger*.4;}

            //robot.armMotorTwo.setPower(gamepad2.left_stick_y);
            //robot.armMotorOne.setPower(-gamepad2.right_stick_y*.5);


            /**
            if(gamepad2.a){
                robot.armServo.setPower(5);
            } else if(gamepad2.b){
                robot.armServo.setPower(-5);
            } else{
                robot.armServo.setPower(0);
            }
            if(gamepad2.x){
                robot.posServo.setPosition(open);
            } else if(gamepad2.y){
                robot.posServo.setPosition(closed);
            }**/
           while(gamepad1.a){
               robot.leftfrontDrive.setPower(60);
            }
            while(gamepad1.b){
                robot.rightfrontDrive.setPower(60);
            }
            while(gamepad1.x){
                robot.leftbackDrive.setPower(60);
            }
            while(gamepad1.y){
                robot.rightbackDrive.setPower(60);
            }
            while(gamepad1.dpad_up){
                robot.leftfrontDrive.setPower(1);
            }
            while(gamepad1.dpad_left){
                robot.rightfrontDrive.setPower(1);
            }
            while(gamepad1.dpad_down){
                robot.leftbackDrive.setPower(1);
            }
            while(gamepad1.dpad_right){
                robot.rightbackDrive.setPower(1);
            }
            testTriggerRight();
            testTriggerLeft();
            telemetry.update();
        }
    }
    public void flipWheelConfigurationBackward(){
        /**
         * negating the power switches the direction:
         * forward is now backward
         * strafe right is now strafe left
         * the turnPower variable is left alone bc it doesn't switch when the wheels switch
         **/
        direction = 1;
    }
    public void flipWheelConfigurationNormal(){
        /**
         * negating the power switches the direction:
         * forward is now backward
         * strafe right is now strafe left
         * the turnPower variable is left alone bc it doesn't switch when the wheels switch
         **/
        direction = -1;
    }
    public void testTriggerRight(){
        telemetry.addData("Right Trigger",gamepad1.right_trigger);
    }
    public void testTriggerLeft(){
        telemetry.addData("Left Trigger",gamepad1.left_trigger);
    }
}