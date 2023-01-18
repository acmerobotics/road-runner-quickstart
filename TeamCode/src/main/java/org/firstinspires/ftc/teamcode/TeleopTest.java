/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop_Test", group="Concept")
@Disabled
public class TeleopTest extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    private final ChassisWith4Motors chassis = new ChassisWith4Motors();

    // Driving motor variables
    static final double HIGH_SPEED_POWER = 0.6;

    //claw and arm unit
    private final ArmClawUnit armClaw = new ArmClawUnit();

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    // voltage management
    LynxModule ctrlHub;
    LynxModule exHub;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        chassis.init(hardwareMap, "FrontLeft", "FrontRight",
                "BackLeft", "BackRight");

        armClaw.init(hardwareMap, "ArmServo", "ClawServo");

        // power control
        ctrlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        exHub = hardwareMap.get(LynxModule.class, "Control Hub");
        double ctrlHubCurrent, ctrlHubVolt, exHubCurrent, exHubVolt, auVolt;
        double maxCtrlCurrent = 0.0, minCtrlVolt = 15.0, maxExCurrent = 0.0, minExVolt = 15.0, minAuVolt = 15.0;


        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double timeStamp = 0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            double maxDrivePower;
            double chassisCurrentPower = chassis.getAveragePower();
            double deltaTime = runtime.milliseconds() - timeStamp;
            double powerRampRate = 0.3 / 100; // 0.3 per 100 ms
            double maxP = chassisCurrentPower + powerRampRate * deltaTime;
            timeStamp = runtime.milliseconds();

            maxDrivePower = Math.min(maxP, HIGH_SPEED_POWER);

            double drive = maxDrivePower * gpButtons.robotDrive;
            double turn  =  maxDrivePower * (-gpButtons.robotTurn);
            double strafe = maxDrivePower * gpButtons.robotStrafe;

            chassis.drivingWithPID(drive, turn, strafe, true);

            // Set position only when button is hit.
            if (gpButtons.clawClose) {
                armClaw.clawClose();
            }


            // Set position only when button is hit.
            if (gpButtons.armLeft) {
                armClaw.setArmPosition(armClaw.ARM_SWING_LEFT);
            }

            // Set position only when button is hit.
            if (gpButtons.armRight) {
                armClaw.setArmPosition(armClaw.ARM_SWING_RIGHT);
            }

            // Set position only when button is hit.
            if (gpButtons.armForward) {
                armClaw.setArmPosition(armClaw.ARM_SWING_FORWARD);
            }

            if (debugFlag) {

                telemetry.addData("ARM", "position = %.2f", armClaw.getArmPosition());
                ctrlHubCurrent = ctrlHub.getCurrent(CurrentUnit.AMPS);
                ctrlHubVolt = ctrlHub.getInputVoltage(VoltageUnit.VOLTS);
                exHubCurrent = exHub.getCurrent(CurrentUnit.AMPS);
                exHubVolt = exHub.getInputVoltage(VoltageUnit.VOLTS);

                auVolt = ctrlHub.getAuxiliaryVoltage(VoltageUnit.VOLTS);
                minAuVolt = Math.min(auVolt, minAuVolt);

                maxCtrlCurrent = Math.max(ctrlHubCurrent, maxCtrlCurrent);
                maxExCurrent = Math.max(exHubCurrent, maxExCurrent);
                minCtrlVolt = Math.min(ctrlHubVolt, minCtrlVolt);
                minExVolt = Math.min(exHubVolt, minExVolt);

                telemetry.addData("Front Center distance sensor", "%.2f", chassis.getFcDsValue());
                telemetry.addData("Front Left distance sensor", "%.2f", chassis.getFlDsValue());
                telemetry.addData("Front Right distance sensor", "%.2f", chassis.getFrDsValue());

                telemetry.addData("Max Ctrl hub current = ", "%.2f", maxCtrlCurrent);
                telemetry.addData("Min Ctrl hub Volt = ", "%.2f", minCtrlVolt);
                telemetry.addData("Max Extend hub current = ", "%.2f", maxExCurrent);
                telemetry.addData("Min Extend hub Volt = ", "%.2f", minExVolt);

                Logging.log("Get drive power = %.2f, set drive power = %.2f", chassisCurrentPower, maxP);
                Logging.log("Ctrl hub current = %.2f, max = %.2f", ctrlHubCurrent, maxCtrlCurrent);
                Logging.log("Ctrl hub volt = %.2f, min = %.2f", ctrlHubVolt, minCtrlVolt);
                Logging.log("Extend hub current = %.2f, max = %.2f", exHubCurrent, maxExCurrent);
                Logging.log("Extend hub volt = %.2f, min = %.2f", exHubVolt, minExVolt);
                Logging.log("Auxiliary voltage = %.2f, min = %.2f", auVolt, minAuVolt);

                // imu log
                chassis.getAngle();
                telemetry.addData("imu heading yaw ", "%.2f", chassis.lastAngles.getYaw(AngleUnit.DEGREES));
                telemetry.addData("imu heading Roll ", "%.2f", chassis.lastAngles.getRoll(AngleUnit.DEGREES));
                telemetry.addData("imu heading pitch ", "%.2f", chassis.lastAngles.getPitch(AngleUnit.DEGREES));
                telemetry.addData("global heading ", "%.2f", chassis.globalAngle);
                telemetry.addData("Correction  ", "%.2f", chassis.correction);

                AngularVelocity angularVelocity = chassis.imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

                // drive motors log
                telemetry.addData("Max driving power ", "%.2f", maxDrivePower);
            }
            // running time
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Status", "While loop Time in ms = ", "%.1f", deltaTime);
            telemetry.update(); // update message at the end of while loop
            Logging.log("While loop time in ms = %.1f.", deltaTime);
        }
        chassis.setPowers(0.0);
    }
}