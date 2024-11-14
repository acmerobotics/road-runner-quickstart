package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * teenageteleop class defines the teleoperated controls for the robot.
 * It handles driving, arm movement, and various servo controls based on gamepad inputs.
 */
@TeleOp(name = "teenage teleop", group = "Pushbot")
public class teenageteleop extends LinearOpMode {
    teenagehwmap robot = new teenagehwmap();
    private ElapsedTime runtime = new ElapsedTime();

    // Power control variables
    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount = 1; // Default speed factor
    static double open = 0.3;
    static double closed = 0.5;
    static double direction = -1; // Direction control
    static double rangeServoDirection = 0.01;
    static double basketServoAmount = 0.01;

    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Drive control
            controlDrive();

            // Gamepad 1 controls
            handleGamepad1Controls();

            // Gamepad 2 controls
            handleGamepad2Controls();

            //
            // Update telemetry
            telemetry.update();
        }
    }

    /** Helper Methods **/

    private void controlDrive() {
        fwdBackPower = direction * -gamepad1.left_stick_y * slowamount;
        strafePower = direction * -gamepad1.left_stick_x * slowamount;
        turnPower = gamepad1.right_stick_x * slowamount;

        lfPower = fwdBackPower - turnPower - strafePower;
        rfPower = fwdBackPower + turnPower + strafePower;
        lbPower = fwdBackPower - turnPower + strafePower;
        rbPower = fwdBackPower + turnPower - strafePower;

        robot.leftfrontDrive.setPower(lfPower);
        robot.leftbackDrive.setPower(lbPower);
        robot.rightfrontDrive.setPower(rfPower);
        robot.rightbackDrive.setPower(rbPower);

        telemetry.addData("Speed Factor", slowamount);
    }

    private void handleGamepad1Controls() {
        // Wheel configuration control
        if (gamepad1.right_bumper) {
            flipWheelConfigurationBackward();
            telemetry.addData("Direction", "Backward");
        } else {
            flipWheelConfigurationNormal();
            telemetry.addData("Direction", "Normal");
        }

        // Slow mode
        if (gamepad1.left_bumper) {
            slowamount = 0.25;
        } else {
            slowamount = 1; // Reset to normal speed
        }

        // Additional controls using gamepad1 buttons
        if (gamepad1.a) {
            robot.leftfrontDrive.setPower(60);
        }
        if (gamepad1.b) {
            robot.rightfrontDrive.setPower(60);
        }
        if (gamepad1.x) {
            robot.leftbackDrive.setPower(60);
        }
        if (gamepad1.y) {
            robot.rightbackDrive.setPower(60);
        }
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
    }

    private void handleGamepad2Controls() {
        robot.range1Servo.setPosition(gamepad2.left_trigger);
        telemetry.addData("servoPos:", robot.range1Servo.getPosition());
        controlIntakeServo();
    }

    private void controlIntakeServo() {
        if (gamepad2.dpad_up) {
            robot.intakeServo.setPower(5);
            telemetry.addData("Intake Power", 5);
        } else if (gamepad2.dpad_down) {
            robot.intakeServo.setPower(-5);
            telemetry.addData("Intake Power", -5);
        } else {
            robot.intakeServo.setPower(0);
            telemetry.addData("Intake Power", 0);
        }
    }

    private void flipWheelConfigurationBackward() {
        direction = 1;
    }

    private void flipWheelConfigurationNormal() {
        direction = -1;
    }

    private void extendRangeServoDirection() {
        rangeServoDirection = robot.Finalrange;
        basketServoAmount = robot.FinalrangeBasket;
    }

    private void resetServos() {
        // Reset basket servo and range servos to neutral positions
        rangeServoDirection = 0;
        basketServoAmount = 0;
        robot.basketServo1.setPosition(basketServoAmount);
        robot.basketServo2.setPosition(basketServoAmount + robot.FinalrangeBasket);
        robot.range1Servo.setPosition(rangeServoDirection);
        robot.range2Servo.setPosition(rangeServoDirection + robot.Finalrange);
        endIntake();
    }

    private void startIntake() {
        robot.intakeServo.setPower(5.0);
        telemetry.addData("Intake", robot.intakeServo.getPower());
    }

    private void endIntake() {
        robot.intakeServo.setPower(0);
        telemetry.addData("Intake", robot.intakeServo.getPower());
    }
}
