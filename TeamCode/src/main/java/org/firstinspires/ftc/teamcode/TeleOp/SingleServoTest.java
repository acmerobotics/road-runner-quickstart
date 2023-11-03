package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SingleServoTest", group="SCC")
public class SingleServoTest extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode() {

        // Connect to servo
        servo = hardwareMap.get(Servo.class, "servo");

        // Wait for the start button
        telemetry.addData(">", "Press Start to test the Servo." );
        telemetry.update();
        waitForStart();

        // Run program until stop pressed.
        while(opModeIsActive()){

            // Calculate the new servo position based on the user's input
            if (gamepad1.left_stick_y > 0 && position <= MAX_POS) {
                position += INCREMENT;
            } else if (gamepad1.left_stick_y < 0 && position >= MIN_POS) {
                position -= INCREMENT;
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause
            servo.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }

        // Program done
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}