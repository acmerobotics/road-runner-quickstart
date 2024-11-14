package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="twinteleop0 with Servo Subroutine", group="Pushbot")
public class twinteleop0 extends LinearOpMode {
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
    static double open = .3;
    static double closed = .5;
    static double direction = -1;
    static double rangeServoDirection = .01;
    static double basketServoAmount = .01;

    // Servo sequence control flag
    private boolean isRoutineRunning = false;

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

            // ---- Gamepad1 D-Pad Controls ----
            if (gamepad1.dpad_up) {
                robot.leftfrontDrive.setPower(1);
            }
            if (gamepad1.dpad_left) {
                robot.rightfrontDrive.setPower(1);
            }
            if (gamepad1.dpad_down) {
                robot.leftbackDrive.setPower(1);
            }
            if (gamepad1.dpad_right) {
                robot.rightbackDrive.setPower(1);
            }

            // ---- Servo Sequence Subroutine Triggered by Gamepad2 ----
            if (gamepad2.a && !isRoutineRunning) {
                // Start the servo sequence in a separate thread
                //isRoutineRunning = true;
                //new Thread(() -> runServoSequence()).start();
            }

            telemetry.update();
        }
    }

    /*
     * Autonomous Servo Sequence
     * Moves servos sequentially when gamepad2.a is pressed
     */
    private void runServoSequence() {
        telemetry.addData("Servo Sequence", "Started");
        telemetry.update();

        // Step 1: Move range1Servo to position 0.25
        robot.range1Servo.setPosition(0.25);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        // Step 2: Move range2Servo to position 0.8
        robot.range2Servo.setPosition(0.8);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        // Step 3: Move range1Servo back to position 0
        robot.range1Servo.setPosition(0);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        // Step 4: Move range2Servo back to position 0
        robot.range2Servo.setPosition(0);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        // --- Step 5: Move Both Servos Simultaneously to 0.25 ---
        robot.range1Servo.setPosition(0.25);
        robot.range2Servo.setPosition(0.25);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        // --- Step 6: Move Both Servos Simultaneously Back to 0 ---
        robot.range1Servo.setPosition(0);
        robot.range2Servo.setPosition(0);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        // Sequence is finished, reset the flag
        isRoutineRunning = false;
    }

    /*
     * Helper Methods
     */
    public void flipWheelConfigurationBackward() {
        direction = 1;  // Change direction to backward
    }

    public void flipWheelConfigurationNormal() {
        direction = -1;  // Change direction to forward
    }

    // Helper sleep method to ensure the OpMode remains active
    private void sleepWithOpModeCheck(long milliseconds) {
        long targetTime = System.currentTimeMillis() + milliseconds;
        while (opModeIsActive() && System.currentTimeMillis() < targetTime) {
            // Wait until the time elapses, keeping OpMode active
        }
    }

    // Testing trigger output for debugging
    public void testTriggerRight() {
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
    }

    public void testTriggerLeft() {
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
    }
}
