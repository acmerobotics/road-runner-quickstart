package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    // New flag for left joystick control during the subroutine
    private boolean isLiftMotorRoutineRunning = false;

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
            if (gamepad2.a && !isRoutineRunning) {
                isRoutineRunning = true;
                new Thread(() -> runFirstServoSequence()).start();  // Execute the servo sequence in a separate thread

            }

            // ---- Second Servo Subroutine (Triggered by gamepad2.b) ----
            if (gamepad2.b && !isRoutineRunning) {
                isRoutineRunning = true;
                new Thread(() -> runSecondServoSequence()).start();  // Execute the servo sequence in a separate thread
            }


            // ---- Trigger the lift motor routine with gamepad2.x ----
            if (gamepad2.x && !isLiftMotorRoutineRunning) {
                isLiftMotorRoutineRunning = true;
                new Thread(() -> runLiftMotorRoutine()).start(); // Run lift motor routine in a separate thread
            }

            telemetry.addData("motorencoder",robot.liftMotor.getCurrentPosition());

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
        moveServosSimultaneously(robot.range1Servo, .25, robot.range2Servo, robot.Finalrange-.25, 0.8);
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

        // Step 1: Move to Zero Position
        moveServosSimultaneously(robot.range1Servo, .15, robot.range2Servo, robot.Finalrange - .15, 0.8);
        if (checkForCancel()) return;

        // Step 2: Move basketServo1 and basketServo2 simultaneously
        moveServosSimultaneously(robot.basketServo1, robot.FinalrangeBasket, robot.basketServo2, 0, 0.8);
        if (checkForCancel()) return;

        // Step 3: Move range1Servo and range2Servo simultaneously
        moveServosSimultaneously(robot.range1Servo, robot.Finalrange, robot.range2Servo, 0, 0.6);
        if (checkForCancel()) return;

        // Wait for gamepad2.left_bumper to be pressed to set retract to true
        while (!gamepad2.left_bumper && opModeIsActive()) {
            telemetry.addData("Waiting for Left Bumper", "Press gamepad2.left_bumper to retract");
            telemetry.update();
            if (checkForCancel()) return;
        }

        retract = true;

        if (retract) {
            // Step 4: Move basketServo1 and basketServo2 to retract positions
            moveServosSimultaneously(robot.basketServo1, 0 + robot.FinalrangeBasket * 0.75, robot.basketServo2, robot.FinalrangeBasket * 0.25, 0.99);
            if (checkForCancel()) return;

            // Step 5: Move range1Servo and range2Servo to final positions
            moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, robot.Finalrange, 0.6);
            if (checkForCancel()) return;

            // Step 6: Return basketServo1 and basketServo2 to starting positions
            moveServosSimultaneously(robot.basketServo1, 0, robot.basketServo2, robot.FinalrangeBasket, 0.99);
            robot.range1Servo.setPosition(0);
            robot.range2Servo.setPosition(robot.Finalrange);
            if (checkForCancel()) return;
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
        if (gamepad2.b) {  // Use gamepad2.b as the cancel button
            telemetry.addData("Sequence", "Canceled by user");
            telemetry.update();
            // Stop the intake servo immediately
            robot.intakeServo.setPower(0);
            isRoutineRunning = false;
            return true;
        }
        return false;
    }


    /*
     * Subroutine to control the lift motor (liffMotor)
     * Changes motor power as requested and ignores left joystick input during execution.
     */
    private void runLiftMotorRoutine() {
        telemetry.addData("Lift Motor Routine", "Started");
        telemetry.update();

        // Get the current position as a baseline
        int initialPosition = robot.liftMotor.getCurrentPosition();

        // Step 1: Move the motor up by 3000 encoder counts at full speed
        int targetPositionMAX = initialPosition + 3080; // Adjust based on desired lift distance
        int targetPositionLowerBasket = 1203; // Adjust based on desired lift distance
        int targetPositionUpperBasket = 2570; // Adjust based on desired lift distance
        int targetPositionLowerRung = 662; // Adjust based on desired lift distance
        int targetPositionUpperRung = 1880; // Adjust based on desired lift distance

        moveMotorToPosition(robot.liftMotor, targetPositionLowerRung,.8);
        robot.liftMotor.setPower(0.05);
        sleepWithOpModeCheck(1000);

        moveMotorToPosition(robot.liftMotor, targetPositionUpperRung,.8);
        robot.liftMotor.setPower(0.05);
        sleepWithOpModeCheck(1000);

        moveMotorToPosition(robot.liftMotor, targetPositionLowerBasket,.8);
        robot.liftMotor.setPower(0.05);
        sleepWithOpModeCheck(1000);

        moveMotorToPosition(robot.liftMotor, targetPositionUpperBasket, .8);

        // Step 2: Hold the position briefly with a low power to maintain
        robot.liftMotor.setPower(0.05);
        sleepWithOpModeCheck(1000);

        moveMotorToPosition(robot.liftMotor, targetPositionMAX, .6);

        // Step 2: Hold the position briefly with a low power to maintain
        robot.liftMotor.setPower(0.05);
        sleepWithOpModeCheck(1000);

        // Step 3: Move the motor down by 3000 encoder counts at 0.4 speed
        int targetPosition2 = initialPosition; // Return to the original position
        moveMotorToPosition(robot.liftMotor, targetPosition2, 0.4);

        // Step 4: Ensure motor is stopped
        robot.liftMotor.setPower(0);

        isLiftMotorRoutineRunning = false; // Re-enable left joystick input
        telemetry.addData("Lift Motor Routine", "Completed");
        telemetry.addData("motorEncoder",robot.liftMotor.getCurrentPosition());
        telemetry.update();
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
            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }

        // Stop the motor once the target is reached
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void controlLiftWithResistance() {
        // Extend the lift until the motor encounters resistance (e.g., reaching max extension)
        telemetry.addData("Lift", "Extending...");
        telemetry.update();
        robot.liftMotor.setPower(1.0); // Set power to extend
        while (opModeIsActive() && !robot.liftMotor.isBusy()) {
            // Simulate resistance detection by checking if the motor is "busy" (stalled or reaching a hard limit)
            // Depending on the hardware, you might need a specific condition here to detect resistance.
            sleep(50);
        }

        // Hold the position briefly after encountering resistance
        telemetry.addData("Lift", "Holding...");
        telemetry.update();
        robot.liftMotor.setPower(0.1); // Small power to hold position
        sleepWithOpModeCheck(500); // Hold for 0.5 seconds

        // Retract the lift until the motor encounters resistance (e.g., fully retracted)
        telemetry.addData("Lift", "Retracting...");
        telemetry.update();
        robot.liftMotor.setPower(-1.0); // Set power to retract
        while (opModeIsActive() && !robot.liftMotor.isBusy()) {
            // Simulate resistance detection again
            sleep(50);
        }

        // Stop the motor when fully retracted
        robot.liftMotor.setPower(0);
        telemetry.addData("Lift", "Fully retracted.");
        telemetry.update();
    }


    private void monitorColorSensor(Color allianceColor) {
        while (opModeIsActive()) {
            float red = robot.colorSensor.red();
            float green = robot.colorSensor.green();
            float blue = robot.colorSensor.blue();

            Color detectedColor = Color.UNKNOWN;

            if (isColorInRange(red, green, blue, Color.RED)) {
                detectedColor = Color.RED;
            } else if (isColorInRange(red, green, blue, Color.GREEN)) {
                detectedColor = Color.GREEN;
            } else if (isColorInRange(red, green, blue, Color.BLUE)) {
                detectedColor = Color.BLUE;
            } else if (isColorInRange(red, green, blue, Color.YELLOW)) {
                detectedColor = Color.YELLOW;
            }

            telemetry.addData("Color Detected", detectedColor);
            adjustIntakePower(allianceColor, detectedColor);
            telemetry.update();

            sleep(50);
        }
    }

    private void adjustIntakePower(Color allianceColor, Color detectedColor) {
        double power;

        if (detectedColor == allianceColor || detectedColor == Color.GREEN || detectedColor == Color.YELLOW) {
            power = 1.0; // Full power for alliance colors
        } else if (detectedColor != allianceColor && detectedColor != Color.UNKNOWN) {
            power = 0; // Stop intake for opposing color
        } else {
            power = 0; // Default to stop
        }

        robot.intakeServo.setPower(power);
    }

    public enum Color {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    private boolean isColorInRange(float red, float green, float blue, Color targetColor) {
        switch (targetColor) {
            case RED:
                return (red > 100 && green < 75 && blue < 75); // Adjust these thresholds as necessary
            case GREEN:
                return (green > 100 && red < 75 && blue < 75); // Adjust these thresholds as necessary
            case BLUE:
                return (blue > 100 && red < 75 && green < 75); // Adjust these thresholds as necessary
            case YELLOW:
                return (red > 100 && green > 100 && blue < 75); // Adjust these thresholds as necessary
            default:
                return false;
        }
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

    private void moveServosDynamically(Servo[] servos, double[] targetPositions, double speedFactor) {
        if (servos.length != targetPositions.length) {
            throw new IllegalArgumentException("Number of servos must match the number of target positions.");
        }

        int numServos = servos.length;
        double[] startPositions = new double[numServos];
        double[] deltas = new double[numServos];

        // Get starting positions and calculate the difference (delta) for each servo
        for (int i = 0; i < numServos; i++) {
            startPositions[i] = servos[i].getPosition();
            deltas[i] = (targetPositions[i] - startPositions[i]) / 100;  // Assume 100 steps for smooth movement
        }

        // Move the servos in steps
        for (int step = 0; step <= 100; step++) {
            for (int i = 0; i < numServos; i++) {
                double newPosition = startPositions[i] + (deltas[i] * step);
                servos[i].setPosition(newPosition);  // Update each servo's position incrementally
            }
            sleep((long) (20 * (1 - speedFactor)));  // Adjust sleep based on speed factor
        }

        // Ensure all servos end at their exact target positions
        for (int i = 0; i < numServos; i++) {
            servos[i].setPosition(targetPositions[i]);
        }
    }

    /**
     * EXAMPLE FOR CODE ABOVE
     * Servo[] servos = {robot.servo1, robot.servo2, robot.servo3, robot.servo4};
     * double[] targetPositions = {0.5, 0.25, 0.75, 1.0};
     * moveServosDynamically(servos, targetPositions, 0.8);  // 80% speed
     */

    private void moveMultipleServosWithSpeeds(Servo[] servos, double[] targetPositions, double[] speedFactors) {
        int numServos = servos.length;

        // Ensure the number of servos matches the number of target positions and speeds
        if (targetPositions.length != numServos || speedFactors.length != numServos) {
            telemetry.addData("Error", "Mismatch in the number of servos, target positions, or speed factors");
            telemetry.update();
            return;
        }

        // Store the initial positions of all servos
        double[] startPositions = new double[numServos];
        for (int i = 0; i < numServos; i++) {
            startPositions[i] = servos[i].getPosition();
        }

        int steps = 100;  // Define the number of steps for smooth movement

        // Calculate deltas for each servo based on its speed
        double[] deltas = new double[numServos];
        for (int i = 0; i < numServos; i++) {
            deltas[i] = (targetPositions[i] - startPositions[i]) / steps;
        }

        // Move all servos in small increments based on the speed factor
        for (int step = 0; step < steps && opModeIsActive(); step++) {
            for (int i = 0; i < numServos; i++) {
                double newPosition = startPositions[i] + (deltas[i] * step * speedFactors[i]);
                servos[i].setPosition(newPosition);
            }

            // Adjust the sleep time to allow slower movements
            sleep(20);
        }

        // Ensure all servos reach their exact target positions
        for (int i = 0; i < numServos; i++) {
            servos[i].setPosition(targetPositions[i]);
        }
    }

    /**
     * EXAMPLE FOR CODE ABOVE
     * moveMultipleServosWithSpeeds(
     *     new Servo[] { robot.servo1, robot.servo2, robot.servo3, robot.servo4 }, // Array of servos
     *     new double[] { 0.25, 0.5, 0.75, 1.0 },                                  // Target positions for each servo
     *     new double[] { 0.8, 0.6, 0.4, 1.0 }                                     // Speed factors for each servo
     * );
     * */

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
