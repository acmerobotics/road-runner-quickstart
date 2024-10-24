package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "twinteleop3 with Servo Subroutines", group = "Pushbot")
public class twinteleop3 extends LinearOpMode {
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
                new Thread(this::runFirstServoSequence).start();  // Execute the servo sequence in a separate thread
            }

            // ---- Second Servo Subroutine (Triggered by gamepad2.b) ----
            if (gamepad2.b && !isRoutineRunning) {
                isRoutineRunning = true;
                new Thread(this::runSecondServoSequence).start();  // Execute the servo sequence in a separate thread
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

            // Start the intake servo if range1Servo is above startIntakePosition
            if (currentPosition1 >= startIntakePosition) {
                robot.intakeServo.setPower(1.0);
            } else {
                robot.intakeServo.setPower(0);
            }

            double currentPosition2 = startPosition2 + (delta2 * i);
            range1Servo.setPosition(currentPosition1);
            range2Servo.setPosition(currentPosition2);
            sleep((long) (10 * speedFactor)); // Adjust sleep time for speed control
        }
    }

    /*
     * General method to move two servos simultaneously
     */
    private void moveServosSimultaneously(Servo servo1, double targetPosition1, Servo servo2, double targetPosition2, double speedFactor) {
        moveServosWithSpecialLogic(servo1, targetPosition1, servo2, targetPosition2, speedFactor);
    }

    /*
     * Method to handle sleep with op mode checks
     */
    private void sleepWithOpModeCheck(long millis) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < millis && opModeIsActive()) {
            sleep(50); // Sleep in smaller increments to allow for op mode checks
        }
    }

    private void flipWheelConfigurationNormal() {
        direction = -1;
    }

    private void flipWheelConfigurationBackward() {
        direction = 1;
    }

    /*
     * New Method to Move Multiple Servos with Speeds
     */
    private void moveMultipleServosWithSpeeds(Servo[] servos, double[] targetPositions, double[] speeds) {
        if (servos.length != targetPositions.length || servos.length != speeds.length) {
            throw new IllegalArgumentException("All input arrays must have the same length.");
        }

        int steps = 100; // Number of steps for smooth movement
        double[] startingPositions = new double[servos.length];

        // Store starting positions
        for (int i = 0; i < servos.length; i++) {
            startingPositions[i] = servos[i].getPosition();
        }

        double[][] deltas = new double[servos.length][steps];
        // Calculate deltas for each servo
        for (int i = 0; i < servos.length; i++) {
            double delta = (targetPositions[i] - startingPositions[i]) / steps;
            for (int j = 0; j < steps; j++) {
                deltas[i][j] = startingPositions[i] + (delta * j);
            }
        }

        for (int i = 0; i < steps && opModeIsActive(); i++) {
            for (int j = 0; j < servos.length; j++) {
                servos[j].setPosition(deltas[j][i]);
            }
            sleep(10); // Control speed of execution
        }
    }

    /**
     * EXAMPLE FOR CODE ABOVE
     * moveMultipleServosWithSpeeds(
     *     new Servo[] { robot.servo1, robot.servo2, robot.servo3, robot.servo4 }, // Array of servos
     *     new double[] { 0.25, 0.5, 0.75, 1.0 },                                  // Target positions for each servo
     *     new double[] { 0.8, 0.6, 0.4, 1.0 }                                     // Speed factors for each servo
     * );
     */

}
