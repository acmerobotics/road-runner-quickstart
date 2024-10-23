package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teenage teleop", group="Pushbot")
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

    // New variables for servo control
    private boolean retract = false;  // Start with retract set to false
    private boolean servoMovementInProgress = false;
    private long servoStartTime = 0;
    private static final int durationMillis = 1000;  // Servo movement duration
    private static final double servoSpeed = 0.8;    // Servo speed as 80% of normal speed

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Drive base code (unchanged)
            fwdBackPower = -gamepad1.left_stick_y * slowamount;
            strafePower = -gamepad1.left_stick_x * slowamount;
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

            // Manual toggle of retract for testing purposes
            if (gamepad2.left_bumper) {
                retract = !retract;  // Toggle retract state
                sleep(500);  // Debounce the button press
            }

            // Servo movement subroutine based on retract flag
            if (!servoMovementInProgress && retract) {
                // Start retract action if retract is true
                servoMovementInProgress = true;
                servoStartTime = System.currentTimeMillis();
            }

            // Update servo positions based on time elapsed
            if (servoMovementInProgress) {
                long currentTime = System.currentTimeMillis();
                double elapsedTime = (double)(currentTime - servoStartTime) / durationMillis;

                if (elapsedTime >= 1.0) {
                    elapsedTime = 1.0;  // Cap at 1.0 to prevent overshooting
                    servoMovementInProgress = false;  // Finish movement after time
                }

                // Calculate new servo positions based on whether retract is true or false
                double range1Position, range2Position;
                if (retract) {
                    // Retract positions
                    range1Position = (1.0 - elapsedTime) * 0.25;  // Move back from 0.25 to 0
                    range2Position = elapsedTime * 0.25;          // Move back from 0 to 0.25
                } else {
                    // Extend positions
                    range1Position = elapsedTime * 0.25;          // Move from 0 to 0.25
                    range2Position = (1.0 - elapsedTime) * 0.25;  // Move from 0.25 to 0
                }

                // Set servo positions with 80% speed
                robot.range1Servo.setPosition(range1Position * servoSpeed);
                robot.range2Servo.setPosition(range2Position * servoSpeed);
            }

            // Telemetry feedback
            telemetry.addData("Servo1 Position", robot.range1Servo.getPosition());
            telemetry.addData("Servo2 Position", robot.range2Servo.getPosition());
            telemetry.addData("Retract Status", retract);
            telemetry.update();
        }
    }
}
