package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@TeleOp(name="teenage teleop", group="Pushbot")
public class teenageteleop extends LinearOpMode {
    teenagehwmap robot=new teenagehwmap();
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
    static double rangeServoDirection = .01;
    static double basketServoAmount = .01;


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
            turnPower = -gamepad1.right_stick_x * slowamount;

            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);


            robot.leftfrontDrive.setPower(lfPower);
            robot.leftbackDrive.setPower(lbPower);
            robot.rightfrontDrive.setPower(rfPower);
            robot.rightbackDrive.setPower(rbPower);

            slowamount = .5;

            /**
             * GAMEPAD1
             * */

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
            else{slowamount = gamepad1.right_trigger*.6;}

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

            /**
             * GAMEPAD2
             * */
            if(gamepad2.a){
                if(rangeServoDirection*1.05<(140/360)){
                    /**ARM Extension**/
                    /**
                     * As the extender motor rotates almost a full 180 degrees to extend the arm,
                     * the basket servo also goes a full 180 degrees to flip out he basket.
                     * Since these amounts mirror each other, they can be extended to the same
                     * position to reach super-extension of the basket and arm at the same time*/
                    robot.range1Servo.setPosition(rangeServoDirection);
                    robot.basketServo1.setPosition(rangeServoDirection);
                    extendRangeServoDirection();

                    if((rangeServoDirection<.25) &&(rangeServoDirection>.125)){
                        startOuttake();
                    }
                    else if(rangeServoDirection>.25){
                        startIntake();
                    }
                }
            }
            if(gamepad2.right_bumper){
                rangeServoDirection = .01;
                robot.range1Servo.setPosition(rangeServoDirection);
            }

            testTriggerRight();
            testTriggerLeft();
            telemetry.update();
        }
    }

    /*
    * Helper Methods*/

    public void flipWheelConfigurationBackward(){
        /**
         * negating the power switches the direction:
         * forward is now backward
         * strafe right is now strafe left
         * the turnPower variable is left alone bc it doesn't switch when the wheels switch
         **/
        direction = -1;
    }
    public void flipWheelConfigurationNormal(){
        /**
         * negating the power switches the direction:
         * forward is now backward
         * strafe right is now strafe left
         * the turnPower variable is left alone bc it doesn't switch when the wheels switch
         **/
        direction = 1;
    }
    public void testTriggerRight(){
        telemetry.addData("Right Trigger",gamepad1.right_trigger);
    }
    public void testTriggerLeft(){
        telemetry.addData("Left Trigger",gamepad1.left_trigger);
    }

    public void extendRangeServoDirection(){
        rangeServoDirection = (100/360);
    }
    public void startOuttake(){
        robot.intakeServo.setPower(-5);
    }
    public void startIntake(){
        robot.intakeServo.setPower(5);
    }
}