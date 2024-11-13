package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="twinteleopScrimmage", group="Pushbot")
public class twinteleopScrimmage extends LinearOpMode {
    teenagehwmap robot = new teenagehwmap();
    private ElapsedTime runtime = new ElapsedTime();

    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount;
    static double direction = -1;
    public boolean clamp = false;
    public boolean liftClamp = false;

    // Servo sequence control flags
    private boolean isRoutineRunning = false;
    private boolean retract = false;  // boolean for retract mode
    private boolean retract2 = false;

    // New flag for left joystick control during the subroutine
    private boolean isLiftMotorRoutineRunning = false;

    //LiftMotorCOnstants
    int initialPosition = 13;
    int targetPositionLowerBasket = 1802; // Adjust based on desired lift distance
    int targetPositionUpperBasket = 2570; // Adjust based on desired lift distance
    int targetPositionLowerRung = 902; // Adjust based on desired lift distance
    int targetPositionUpperRung = 3080; // Adjust based on desired lift distance

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---- Drive Control ----
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

            slowamount = 1;

            // ---- Mecanum Drive Adjustments ----
            if (gamepad1.right_bumper) {
                flipWheelConfigurationBackward();
                telemetry.addData("Direction-Backward:", direction);
            } else {
                flipWheelConfigurationNormal();
                telemetry.addData("Direction-Normal:", direction);
            }

            if (gamepad1.left_bumper) {
                slowamount = .25;
            }


            if(!isLiftMotorRoutineRunning){
                robot.liftMotor.setPower(gamepad2.left_stick_y);
                telemetry.addData("liftMotorPower",robot.liftMotor.getPower());
            }

            // ---- First Servo Sequence (Triggered by gamepad2.a) ----


            if(gamepad2.x){clamp = !clamp;sleepWithOpModeCheck(250);}
            if(clamp){robot.clawServo.setPosition(.19);}
            else{robot.clawServo.setPosition(0);}
            //extender
            moveServosSimultaneously(robot.range1Servo,0+ robot.Finalrange*gamepad2.right_trigger, robot.range2Servo, robot.Finalrange-robot.Finalrange*gamepad2.right_trigger, 1);
            if(gamepad2.y){liftClamp = !liftClamp;sleepWithOpModeCheck(250);}
            if(liftClamp){
                rotateClaw();
            }
            else{
                rotateClaw2();
            }


            // ---- Second Servo Subroutine (Triggered by gamepad2.b) ----
            if (gamepad2.b && !isRoutineRunning) {
                isRoutineRunning = true;
                // // Adjust the power as needed
                new Thread(() -> runSecondServoSequence()).start();  // Execute the servo sequence in a separate thread
                //robot.intakeServo.setPower(0); // Adjust the power as needed
            }


            if(gamepad2.dpad_up){


                // Step 1: Move the motor up by 3000 encoder counts at full speed

                moveMotorToPosition(robot.liftMotor, targetPositionUpperBasket,.8);
                robot.liftMotor.setPower(0);

            }
            else if(gamepad2.dpad_right){


                // Step 1: Move the motor up by 3000 encoder counts at full speed

                moveMotorToPosition(robot.liftMotor, targetPositionUpperRung,.8);
                robot.liftMotor.setPower(0);

            }
            else if(gamepad2.dpad_left){


                // Step 1: Move the motor up by 3000 encoder counts at full speed

                moveMotorToPosition(robot.liftMotor, targetPositionLowerRung,.8);
                robot.liftMotor.setPower(0);

            }
            else if(gamepad2.dpad_down){

                // Step 1: Move the motor up by 3000 encoder counts at full speed

                moveMotorToPosition(robot.liftMotor, initialPosition,.8);
                robot.liftMotor.setPower(0);

            }


            telemetry.addData("intake power", robot.intakeServo.getPower());
            telemetry.addData("claw",robot.clawServo.getPosition());
            telemetry.addData("RotateClaw",robot.clawRotateServo.getPosition()-robot.FinalrangeClawRotate);
            telemetry.addData("LIFT",robot.liftMotor.getCurrentPosition());
            telemetry.addData("LiftClamp",liftClamp);
            telemetry.update();
        }
    }
    /*
     * Second Autonomous Servo Sequence
     * Moves servos based on retract flag triggered by gamepad2.b
     */
    private void runSecondServoSequence() {
        telemetry.addData("Second Servo Sequence", "Started");
        telemetry.update();
        robot.intakeServo.setPower(-1.0);

        //moveServosSimultaneously(robot.basketServo1, 0, robot.basketServo2, robot.FinalrangeBasket, 0.99);
        //moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, robot.Finalrange, 0.6);
        /**
         moveMultipleServosWithSpeeds(
         new Servo[] { robot.range1Servo, robot.range2Servo, robot.basketServo1, robot.basketServo2 },
         new double[] { 0, robot.Finalrange, 0, robot.FinalrangeBasket },
         new double[] { 0.6, 0.6, 0.7, 0.7 }
         );
         **/

        if (checkForCancel()) {isRoutineRunning = false;return;}

        // Start the intake servo when the second servo sequence starts



        // Step 1: Move to Zero Position
        //moveServosSimultaneously(robot.range1Servo, .15, robot.range2Servo, robot.Finalrange - .15, 0.8);
        if (checkForCancel()) {isRoutineRunning = false;return;}
        //sleepWithOpModeCheck(10000);

        // Step 2: Move basketServo1 and basketServo2 simultaneously
        moveServosSimultaneously(robot.basketServo1, robot.FinalrangeBasket, robot.basketServo2, 0, 0.8);
        if (checkForCancel()) {isRoutineRunning = false;return;}

        // Step 3: Move range1Servo and range2Servo simultaneously
        //moveServosSimultaneously(robot.range1Servo, robot.Finalrange, robot.range2Servo, 0, 0.6);
        if (checkForCancel()) {isRoutineRunning = false;return;}
        // Adjust the power as needed

        // Wait for gamepad2.left_bumper to be pressed to set retract to true
        while (!gamepad2.left_bumper && opModeIsActive()) {
            robot.intakeServo.setPower(-1.0);
            telemetry.addData("Waiting for Left Bumper", "Press gamepad2.left_bumper to retract");
            telemetry.update();
            if (checkForCancel()) return;
        }

        retract = true;

        if (retract) {
            robot.intakeServo.setPower(-1.0);
            // Step 4: Move basketServo1 and basketServo2 to retract positions
            moveServosSimultaneously(robot.basketServo1, 0 + robot.FinalrangeBasket * 0.75, robot.basketServo2, robot.FinalrangeBasket * 0.25, 0.99);
            if (checkForCancel()) {isRoutineRunning = false;return;}
            sleepWithOpModeCheck(500);

            // Step 5: Move range1Servo and range2Servo to final positions
            //moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, robot.Finalrange, 0.6);
            if (checkForCancel()) {isRoutineRunning = false;return;}

            // Step 6: Return basketServo1 and basketServo2 to starting positions

            //robot.range1Servo.setPosition(0);
            //robot.range2Servo.setPosition(robot.Finalrange);

            if (checkForCancel()) {isRoutineRunning = false;return;}

        }

        // Wait for gamepad2.left_bumper to be pressed to set retract to true
        while (!gamepad2.left_bumper && opModeIsActive()) {
            robot.intakeServo.setPower(-1.0);
            telemetry.addData("Waiting for Left Bumper", "Press gamepad2.left_bumper to retract");
            telemetry.update();
            if (checkForCancel()) return;
        }
        retract2 = true;
        if(retract2){
            robot.intakeServo.setPower(-1.0);
            moveServosSimultaneously(robot.basketServo1, 0, robot.basketServo2, robot.FinalrangeBasket, 0.99);
            if (checkForCancel()) {isRoutineRunning = false;return;}
        }

        sleepWithOpModeCheck(1000);

        // Stop the intake servo when the second servo sequence ends
        robot.intakeServo.setPower(0); // Stop the intake servo

        isRoutineRunning = false;
        telemetry.addData("Second Servo Sequence", "Completed");
        telemetry.update();
    }
    /**
     * Checks if the cancel button (gamepad2.b) is pressed and stops the routine if true.
     * @return true if canceled, false otherwise
     */
    private boolean checkForCancel() {
        if (gamepad2.right_bumper) {  // Use gamepad2.b as the cancel button
            telemetry.addData("Sequence", "Canceled by user");
            telemetry.update();
            // Stop the intake servo immediately
            robot.intakeServo.setPower(0);
            isRoutineRunning = false;
            return true;
        }
        return false;
    }
    /**
     * Moves a DC motor to a target position with a specified speed.
     *
     * @param motor The DC motor to control.
     * @param targetPosition The encoder target position to move to.
     * @param speed The motor power (0.0 to 1.0) used to reach the target.
     */
    private void moveMotorToPosition(DcMotor motor, int targetPosition, double speed) {
        // Set the target position and configure the motor to run to it
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor power (speed) while ensuring it’s non-negative
        motor.setPower(Math.abs(speed));

        // Wait until the motor reaches the target position
        while (motor.isBusy()) {
            // You can add telemetry or perform other tasks here while waiting
            //telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }

        // Stop the motor once the target is reached
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /*
     * Helper method to move two servos simultaneously
     * targetPosition1: Target for servo1, targetPosition2: Target for servo2
     * speedFactor should be between 0 and 1, where 1 is 100% speed
     */
    private void moveServosSimultaneously(Servo servo1, double targetPosition1, Servo servo2, double targetPosition2, double speedFactor) {
        double startPosition1 = servo1.getPosition();
        double startPosition2 = servo2.getPosition();

        int steps = 100; // Number of steps for smooth movement
        double delta1 = (targetPosition1 - startPosition1) / steps;
        double delta2 = (targetPosition2 - startPosition2) / steps;

        for (int i = 0; i < steps; i++) {
            if (checkForCancel()) return;
            servo1.setPosition(startPosition1 + (delta1 * i));
            servo2.setPosition(startPosition2 + (delta2 * i));
            sleep((long) (20 * (1 - speedFactor))); // Sleep adjusted based on speed factor
        }

        // Ensure both servos end at their exact target positions
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);
    }
    private void moveServoToPosition(Servo servo, double targetPosition, double speedFactor) {
        double startPosition = servo.getPosition();

        int steps = 100; // Number of steps for smooth movement
        double delta = (targetPosition - startPosition) / steps;

        for (int i = 0; i < steps; i++) {
            if (checkForCancel()) return;
            servo.setPosition(startPosition + (delta * i));
            sleep((long) (20 * (1 - speedFactor))); // Sleep adjusted based on speed factor
        }

        // Ensure the servo ends at the exact target position
        servo.setPosition(targetPosition);
    }
    public void rotateClaw(){
        moveServoToPosition(robot.clawRotateServo,robot.FinalposClawRotate,.8);
    }
    public void rotateClaw2(){
        moveServoToPosition(robot.clawRotateServo,robot.FinalrangeClawRotate,.8);
    }
    // ---- Drive Configuration Methods ----
    public void flipWheelConfigurationBackward() {
        direction = 1;
    }

    public void flipWheelConfigurationNormal() {
        direction = -1;
    }

    private void sleepWithOpModeCheck(long milliseconds) {
        long endTime = System.currentTimeMillis() + milliseconds;
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            // Do nothing, just wait
        }
    }
}
