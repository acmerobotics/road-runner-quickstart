package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop.NeedToTest;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Blinkin")

@Disabled
public class ZBlinkin extends LinearOpMode {

    private RevBlinkinLedDriver BlinkyLight;

    RevBlinkinLedDriver.BlinkinPattern pattern;
    boolean autoDisplay;
    ElapsedTime gamepadTimer;
    ElapsedTime displayTimer;

    /**
     * This function sends telemetry info to Driver Station.
     */
    private void doTelemetry() {
        telemetry.addData(">>", "Press A for Manual Mode, Press B for Auto Mode.");
        telemetry.addData("Auto Display Mode", autoDisplay);
        telemetry.addData("Pattern", pattern.toString());
        if (!autoDisplay) {
            telemetry.addData("   >>", "Left Bumper = Previous, Right Bumper = Next");
        }
    }

    private void InitialSetup() {
        // Sets color pattern pallete
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        BlinkyLight.setPattern(pattern);
        // If autoDisplay is true, then automatically cycle through blink patterns
        autoDisplay = true;
        telemetry.addData(">>", "Push start to run op mode...");
        telemetry.addData("Auto Display Mode", autoDisplay);
        telemetry.addData("Pattern", pattern.toString());
        telemetry.update();
    }

    /**
     * This function will check to see if the user wants to
     * toggle between auto display mode and manual mode.
     * if it's in manual mode, the function will increment or
     * decrement the blink pattern based on the button presses.
     */
    private void handleGamepad() {
        // Don't handle gamepad unless timer exceeds limit.
        if (gamepadTimer.milliseconds() > 500) {
            if (gamepad1.a) {
                // switch to manual mode.
                autoDisplay = false;
                gamepadTimer.reset();
            } else if (gamepad1.b) {
                if (!autoDisplay) {
                    // Reset display timer.
                    displayTimer.reset();
                }
                // switch to auto display mode.
                autoDisplay = true;
                gamepadTimer.reset();
            } else if (!autoDisplay) {
                // manual mode
                if (gamepad1.left_bumper) {
                    // change to previous pattern.
                    pattern = pattern.previous();
                    BlinkyLight.setPattern(pattern);
                    gamepadTimer.reset();
                } else if (gamepad1.right_bumper) {
                    // change to next pattern.
                    pattern = pattern.next();
                    BlinkyLight.setPattern(pattern);
                    gamepadTimer.reset();
                }
            }
        }
    }

    // This function will cycle the pattern
    private void doAutoDisplay() {
        // does display timer exceed limit?
        if (displayTimer.seconds() > 5) {
            // move to the next pattern
            pattern = pattern.next();
            BlinkyLight.setPattern(pattern);
            displayTimer.reset();
        }
    }

    @Override
    public void runOpMode() {
        BlinkyLight = hardwareMap.get(RevBlinkinLedDriver.class, "BlinkyLight");

        // This op mode shows how to use the Blinkin LED Controller.
        // Initialize the system.
        InitialSetup();
        // Wait for user to push start button.
        waitForStart();
        // Check to make sure op mode is still active.
        if (opModeIsActive()) {
            // Start timer to lock out repeated button pushes.
            gamepadTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            // Start timer to auto cycle through patterns
            displayTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            // Loop while the op mode is still active.
            while (opModeIsActive()) {
                // Handle gamepad input.
                handleGamepad();
                // Check to see if in auto display mode.
                if (autoDisplay) {
                    // Is it time to toggle to next pattern?
                    doAutoDisplay();
                }
                // Display telemetry info.
                doTelemetry();
                telemetry.update();
            }
        }
    }


}
