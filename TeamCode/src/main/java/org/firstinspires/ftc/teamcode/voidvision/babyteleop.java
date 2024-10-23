package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="baby teleop", group="Pushbot")
public class babyteleop extends LinearOpMode {

    // Robot hardware map instance
    babyhwmap robot = new babyhwmap();

    // Timer for tracking elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Variables for driving and controlling motors
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
    static double direction = -1;  // Direction modifier for forward/reverse
    static double rangeServoDirection = .01;  // Servo position for range extension

    // Flag to track state changes
    private boolean changed1 = false;

    @Override
    public void runOpMode() {
        // Initialize the robot hardware map
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the driver to start the teleop
        waitForStart();

        // Main control loop while the OpMode is active
        while (opModeIsActive()) {

            // Drive control based on gamepad input
            fwdBackPower = direction * -gamepad1.left_stick_y * slowamount;
            strafePower = direction * -gamepad1.left_stick_x * slowamount;
            turnPower = -gamepad1.right_stick_x * slowamount;

            // Calculating power for each wheel
            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);

            // Setting motor powers for mecanum drive
            robot.leftfrontDrive.setPower(lfPower);
            robot.leftbackDrive.setPower(lbPower);
            robot.rightfrontDrive.setPower(rfPower);
            robot.rightbackDrive.setPower(rbPower);

            // Setting speed modifier for slow mode
            slowamount = 0.5;

            // Flip wheel configuration when right bumper is pressed
            if (gamepad1.right_bumper) {
                flipWheelConfigurationBackward();
                telemetry.addData("Direction", "Backward");
            } else {
                flipWheelConfigurationNormal();
                telemetry.addData("Direction", "Normal");
            }

            // Adjust slow mode speed when left bumper is pressed
            if (gamepad1.left_bumper) {
                slowamount = 0.25;
            } else {
                slowamount = 0.6;
            }

            // Uncommented arm motor controls for future use if necessary
            //robot.armMotorTwo.setPower(gamepad2.left_stick_y);
            //robot.armMotorOne.setPower(-gamepad2.right_stick_y * 0.5);

            // Setting individual motor powers based on button presses
            while (gamepad1.a) {
                robot.leftfrontDrive.setPower(60);
            }
            while (gamepad1.b) {
                robot.rightfrontDrive.setPower(60);
            }
            while (gamepad1.x) {
                robot.leftbackDrive.setPower(60);
            }
            while (gamepad1.y) {
                robot.rightbackDrive.setPower(60);
            }

            // Example DPAD input to control motors
            while (gamepad1.dpad_up) {
                robot.leftfrontDrive.setPower(1);
            }
            while (gamepad1.dpad_left) {
                robot.rightfrontDrive.setPower(1);
            }
            while (gamepad1.dpad_down) {
                robot.leftbackDrive.setPower(1);
            }
            while (gamepad1.dpad_right) {
                robot.rightbackDrive.setPower(1);
            }

            // Uncommented servo control for future use
            /*
            while (gamepad2.x) {
                robot.servo.setPosition(0.25);
            }
            while (gamepad2.y) {
                robot.servo.setPosition(0.5);
            }
            while (gamepad2.b) {
                robot.servo.setPosition(0.6);
            }
            while (gamepad2.a) {
                if (rangeServoDirection * 1.05 < (140 / 360)) {
                    robot.servo.setPosition(rangeServoDirection);
                    telemetry.addData("ServoPosition", robot.servo.getPosition());
                    extendRangeServoDirection();
                }
            }
            if (gamepad2.right_bumper) {
                rangeServoDirection = 0.01;
                robot.servo.setPosition(rangeServoDirection);
            }
            */

            // Test trigger inputs for debugging
            testTriggerRight();
            testTriggerLeft();

            telemetry.update();
        }
    }

    /**
     * Flips the wheel configuration for backward driving.
     * Forward becomes backward, strafe directions also swap.
     */
    public void flipWheelConfigurationBackward() {
        direction = -1;
    }

    /**
     * Resets the wheel configuration to normal driving.
     */
    public void flipWheelConfigurationNormal() {
        direction = 1;
    }

    /**
     * Displays the value of the right trigger for debugging.
     */
    public void testTriggerRight() {
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
    }

    /**
     * Displays the value of the left trigger for debugging.
     */
    public void testTriggerLeft() {
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
    }

    /**
     * Gradually increases the servo's position by multiplying it.
     */
    public void extendRangeServoDirection() {
        rangeServoDirection *= 1.05;
    }
}
