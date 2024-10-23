package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="twinteleop2 with Servo Subroutines", group="Pushbot")
public class twinteleop2 extends LinearOpMode {
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

    // Servo sequence control flags
    private boolean isRoutineRunning = false;
    private boolean retract = false;  // boolean for retract mode

    // Positions
    static final double startIntakePosition = 0.12;
    static final double lowerBasketPosition = 0.25;

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


            // ---- First Servo Sequence (Triggered by gamepad2.a) ----
            if (gamepad2.a && !isRoutineRunning) {
                isRoutineRunning = true;
                new Thread(() -> runFirstServoSequence()).start();  // Execute the servo sequence in a separate thread
            }

            // ---- Second Servo Subroutine (Triggered by gamepad2.b) ----
            if (gamepad2.b && !isRoutineRunning) {
                isRoutineRunning = true;
                new Thread(() -> runSecondServoSequence()).start();  // Execute the servo sequence in a separate thread
            }

            telemetry.update();
        }
    }

    /*
     * First Autonomous Servo Sequence
     * Moves servos simultaneously when gamepad2.a is pressed
     */
    private void runFirstServoSequence() {
        telemetry.addData("First Servo Sequence", "Started");
        telemetry.update();

        // Step 1: Move range1Servo to position 0.25 and range2Servo to position 0.8 at 80% speed
        moveServosSimultaneously(robot.range1Servo, 0.25, robot.range2Servo, 0.8, 0.8);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        // Step 2: Move both servos back to their starting positions at the same time
        moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, 0, 0.8);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        isRoutineRunning = false;
    }

    /*
     * Second Autonomous Servo Sequence
     * Moves servos based on retract flag triggered by gamepad2.b
     */
    private void runSecondServoSequence() {
        telemetry.addData("Second Servo Sequence", "Started");
        telemetry.update();

        // Start the intake servo when the second servo sequence starts
        robot.intakeServo.setPower(1.0); // Adjust the power as needed

        // Step 1: Move range1Servo to 0.25 and range2Servo to 0 at the same time at 80% speed
        moveServosSimultaneously(robot.range1Servo, 0.25, robot.range2Servo, 0, 0.8);

        // Simultaneously move basketServo1 to 0.5 and basketServo2 to 0
        moveServosSimultaneously(robot.basketServo1, 0.5, robot.basketServo2, 0, 0.8);

        // Wait for gamepad2.left_bumper to be pressed to set retract to true
        while (!gamepad2.left_bumper && opModeIsActive()) {
            telemetry.addData("Waiting for Left Bumper", "Press gamepad2.left_bumper to retract");
            telemetry.update();
        }

        retract = true;

        if (retract) {
            // Step 2: Move basketServo1 back to 0 and basketServo2 to 0.5 simultaneously
            moveServosSimultaneously(robot.basketServo1, 0, robot.basketServo2, 0.5, 0.75);

            // Step 3: Move range1Servo back to 0 and range2Servo to 0.25 at the same time at 80% speed
            moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, 0.25, 0.8);
        }

        // Stop the intake servo when the second servo sequence ends
        robot.intakeServo.setPower(0); // Stop the intake servo

        isRoutineRunning = false;
    }


    /*
     * Helper method to move servos with additional logic for intake and basket
     * The intakeServo starts when range1Servo reaches startIntakePosition,
     * and the basket servos move when range1Servo reaches lowerBasketPosition.
     */
    private void moveServosWithSpecialLogic(Servo range1Servo, double targetPosition1, Servo range2Servo, double targetPosition2, double speedFactor) {
        double startPosition1 = range1Servo.getPosition();
        double startPosition2 = range2Servo.getPosition();

        int steps = 100; // Number of steps for smooth movement
        double delta1 = (targetPosition1 - startPosition1) / steps;
        double delta2 = (targetPosition2 - startPosition2) / steps;

        for (int i = 0; i < steps && opModeIsActive(); i++) {
            double currentPosition1 = startPosition1 + (delta1 * i);

            // Move range1Servo and range2Servo
            range1Servo.setPosition(currentPosition1);
            range2Servo.setPosition(startPosition2 + (delta2 * i));

            // Check if range1Servo has reached the startIntakePosition
            if (currentPosition1 >= startIntakePosition && robot.intakeServo.getPower() == 0) {
                telemetry.addData("Intake", "Starting intake at position " + currentPosition1);
                robot.intakeServo.setPower(5.0);
            }

            // Check if range1Servo has reached lowerBasketPosition
            if (currentPosition1 >= lowerBasketPosition && robot.basketServo1.getPosition() != 0.5 && robot.basketServo2.getPosition() != 0) {
                telemetry.addData("Basket", "Lowering basket at position " + currentPosition1);
                moveServosSimultaneously(robot.basketServo1, 0.5, robot.basketServo2, 0, 0.7);
            }

            sleep((long) (20 * (1 - speedFactor))); // Adjust sleep for smooth movement
            telemetry.update();
        }

        // Ensure both servos end at their exact target positions
        range1Servo.setPosition(targetPosition1);
        range2Servo.setPosition(targetPosition2);
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
            servo1.setPosition(startPosition1 + (delta1 * i));
            servo2.setPosition(startPosition2 + (delta2 * i));
            sleep((long) (20 * (1 - speedFactor))); // Sleep adjusted based on speed factor
        }

        // Ensure both servos end at their exact target positions
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);
    }

    /*
     * Helper method to move four servos simultaneously (for retract)
     */
    private void moveMultipleServosSimultaneously(Servo servo1, double targetPosition1, Servo servo2, double targetPosition2,
                                                  Servo servo3, double targetPosition3, Servo servo4, double targetPosition4, double speedFactor) {
        double startPosition1 = servo1.getPosition();
        double startPosition2 = servo2.getPosition();
        double startPosition3 = servo3.getPosition();
        double startPosition4 = servo4.getPosition();

        int steps = 100;
        double delta1 = (targetPosition1 - startPosition1) / steps;
        double delta2 = (targetPosition2 - startPosition2) / steps;
        double delta3 = (targetPosition3 - startPosition3) / steps;
        double delta4 = (targetPosition4 - startPosition4) / steps;

        for (int i = 0; i < steps; i++) {
            servo1.setPosition(startPosition1 + (delta1 * i));
            servo2.setPosition(startPosition2 + (delta2 * i));
            servo3.setPosition(startPosition3 + (delta3 * i));
            servo4.setPosition(startPosition4 + (delta4 * i));
            sleep((long) (20 * (1 - speedFactor)));
        }

        // Ensure all servos end at their exact target positions
        servo1.setPosition(targetPosition1);
        servo2.setPosition(targetPosition2);
        servo3.setPosition(targetPosition3);
        servo4.setPosition(targetPosition4);
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
