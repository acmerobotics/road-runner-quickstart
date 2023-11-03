package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DroneLauncherOneTest", group="SCC")
public class DroneLauncherOneTest extends LinearOpMode {
    static final long SERVO_FULL_ROTATION_DELAY_MS   = 700;

    // Define class members
    Servo   servo;
    double  startPosition = 1.0; // Start position
    double  endPosition = 0.0; // End position

    @Override
    public void runOpMode() {
        // Connect to servo
        servo = hardwareMap.get(Servo.class, "servo");

        // Set the servo start position
        servo.setPosition(startPosition);
        sleep(500);

        // Wait for the start button
        telemetry.addData(">", "Press Start to run the drone launcher test 1.0." );
        telemetry.update();
        waitForStart();

        // Run program until stop pressed.
        while(opModeIsActive()){
            // Launch drone based on the user's input
            if (gamepad1.a) {
                servo.setPosition(endPosition);
                sleep(SERVO_FULL_ROTATION_DELAY_MS);
            } else {
                servo.setPosition(startPosition);
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            idle();
        }

        // Program done
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}