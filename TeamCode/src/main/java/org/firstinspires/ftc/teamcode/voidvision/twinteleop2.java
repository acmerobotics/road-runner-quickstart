package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teenage teleop", group="Pushbot")
public class twinteleop2 extends LinearOpMode {
    teenagehwmap robot = new teenagehwmap();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime(); // Timer to track servo movements

    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double slowamount;
    static double direction = -1;

    static double range1ServoTarget = 0;
    static double range2ServoTarget = 0;
    private boolean retract = false;
    private boolean servoMovementInProgress = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status,", "Ready to run");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ---- Drive Control (gamepad1) ----
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

            // Speed control (gamepad1.left_bumper)
            slowamount = gamepad1.left_bumper ? 0.25 : 1;

            // ---- Servo Control (gamepad2) ----
            if (gamepad2.b && !servoMovementInProgress) {
                // Start the retract action
                range1ServoTarget = 0.25;
                range2ServoTarget = 0.0;
                retract = false;
                servoMovementInProgress = true;
                servoTimer.reset(); // Start the timer for the movement
            }

            if (gamepad2.left_bumper && servoMovementInProgress) {
                // Switch retract state
                retract = true;
                range1ServoTarget = 0.0;
                range2ServoTarget = 0.25;
                servoTimer.reset(); // Reset timer to track retract movement
            }

            // Continuously monitor servo movement (non-blocking)
            if (servoMovementInProgress) {
                moveServosNonBlocking(robot.range1Servo, robot.range2Servo, range1ServoTarget, range2ServoTarget, servoTimer, 1000);

                // End the movement when done
                if (servoTimer.milliseconds() > 1000) { // Assuming the move takes 1 second
                    servoMovementInProgress = false;
                }
            }

            telemetry.addData("Servo Movement:", servoMovementInProgress);
            telemetry.addData("Range1Servo Position:", robot.range1Servo.getPosition());
            telemetry.addData("Range2Servo Position:", robot.range2Servo.getPosition());
            telemetry.addData("Drive Powers (lf, lb, rf, rb):", "%f, %f, %f, %f", lfPower, lbPower, rfPower, rbPower);
            telemetry.update();
        }
    }

    /**
     * Non-blocking method to move servos gradually over time.
     */
    private void moveServosNonBlocking(Servo servo1, Servo servo2, double targetPosition1, double targetPosition2, ElapsedTime timer, long durationMillis) {
        double elapsed = timer.milliseconds();
        double fraction = Math.min(elapsed / durationMillis, 1.0); // Clamp fraction to 1.0 (done)

        // Calculate intermediate positions based on the fraction of time elapsed
        double currentPosition1 = servo1.getPosition();
        double currentPosition2 = servo2.getPosition();
        double newPosition1 = currentPosition1 + (targetPosition1 - currentPosition1) * fraction;
        double newPosition2 = currentPosition2 + (targetPosition2 - currentPosition2) * fraction;

        // Set new positions to the servos
        servo1.setPosition(newPosition1);
        servo2.setPosition(newPosition2);
    }
}
