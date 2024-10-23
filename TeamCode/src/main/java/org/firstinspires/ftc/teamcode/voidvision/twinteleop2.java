package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teenage teleop with Servo Subroutines", group = "Pushbot")
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

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Control for gamepad1 (Driving)
            controlDrive();

            // Control for gamepad2 (Servos)
            controlServos();

            telemetry.update();
        }
    }

    // ---- Drive Control Method ----
    private void controlDrive() {
        // Read gamepad1 inputs for driving
        double fwdBackPower = direction * -gamepad1.left_stick_y * slowamount;
        double strafePower = direction * -gamepad1.left_stick_x * slowamount;
        double turnPower = gamepad1.right_stick_x * slowamount;

        // Calculate power for each motor
        lfPower = (fwdBackPower - turnPower - strafePower);
        rfPower = (fwdBackPower + turnPower + strafePower);
        lbPower = (fwdBackPower - turnPower + strafePower);
        rbPower = (fwdBackPower + turnPower - strafePower);

        // Set motor powers
        robot.leftfrontDrive.setPower(lfPower);
        robot.leftbackDrive.setPower(lbPower);
        robot.rightfrontDrive.setPower(rfPower);
        robot.rightbackDrive.setPower(rbPower);

        // Adjust slow mode
        slowamount = gamepad1.left_bumper ? 0.25 : 1;

        // Wheel configuration control
        if (gamepad1.right_bumper) {
            flipWheelConfigurationBackward();
        } else {
            flipWheelConfigurationNormal();
        }
    }

    // ---- Servo Control Method ----
    private void controlServos() {
        // Execute servo routines based on gamepad2 inputs
        if (gamepad2.a && !isRoutineRunning) {
            isRoutineRunning = true;
            runFirstServoSequence();
        }

        if (gamepad2.b && !isRoutineRunning) {
            isRoutineRunning = true;
            runSecondServoSequence();
        }
    }

    // ---- First Servo Sequence ----
    private void runFirstServoSequence() {
        telemetry.addData("First Servo Sequence", "Started");
        telemetry.update();

        moveServosSimultaneously(robot.range1Servo, 0.25, robot.range2Servo, 0.8, 0.8);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, 0, 0.8);
        sleepWithOpModeCheck(1000); // Wait for 1 second

        isRoutineRunning = false;
    }

    // ---- Second Servo Sequence ----
    private void runSecondServoSequence() {
        telemetry.addData("Second Servo Sequence", "Started");
        telemetry.update();

        moveServosSimultaneously(robot.range1Servo, 0.25, robot.range2Servo, 0, 0.8);

        while (!gamepad2.left_bumper && opModeIsActive()) {
            telemetry.addData("Waiting for Left Bumper", "Press gamepad2.left_bumper to retract");
            telemetry.update();
        }

        moveServosSimultaneously(robot.range1Servo, 0, robot.range2Servo, 0.25, 0.8);
        isRoutineRunning = false;
    }

    // ---- Helper Method to Move Servos Simultaneously ----
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

    // ---- Sleep Method with OpMode Check ----
    private void sleepWithOpModeCheck(long milliseconds) {
        long endTime = System.currentTimeMillis() + milliseconds;
        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            // Do nothing, just wait
        }
    }

    // ---- Drive Configuration Methods ----
    public void flipWheelConfigurationBackward() {
        direction = 1;
    }

    public void flipWheelConfigurationNormal() {
        direction = -1;
    }
}
