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
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /*
  * PID controller and IMU codes are copied from
  * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
  */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

@TeleOp(name="Teleop_DualDrivers", group="Concept")
//@Disabled
public class TeleopDualDrivers extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    private final ChassisWith4Motors chassis = new ChassisWith4Motors();

    // Driving motor variables
    final double POWER_LOW = 0.3;
    final double POWER_NORMAL = 0.6;
    final double POWER_HIGH = 0.95;

    // slider motor power variables
    private final SlidersWith2Motors slider = new SlidersWith2Motors();

    //claw and arm unit
    private final ArmClawUnit armClaw = new ArmClawUnit();

    // variables for auto load and unload cone
    double moveOutJunctionDistance = 5.0; // in INCH

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = false;

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
        slider.init(hardwareMap, "RightSlider", "LeftSlider");
        slider.setInchPosition(Params.GROUND_CONE_POSITION);

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

        // move slider to wall position just when starting.
        if (opModeIsActive()) {
            slider.setInchPosition(Params.WALL_POSITION);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            // update chassis power
            double maxDrivePower;
            double chassisCurrentPower = chassis.getAveragePower();
            double deltaTime = runtime.milliseconds() - timeStamp;
            double powerRampRate = 0.3 / 100; // 0.3 per 100 ms for speed ramp up
            double maxP = chassisCurrentPower + powerRampRate * deltaTime;
            timeStamp = runtime.milliseconds();

            if (gpButtons.speedUp) {
                maxDrivePower = POWER_HIGH;
            } else if (gpButtons.speedDown) {
                maxDrivePower = POWER_LOW;
            } else {
                maxDrivePower = POWER_NORMAL;
            }

            maxDrivePower = Math.min(maxP, maxDrivePower);

            double drive = maxDrivePower * gpButtons.robotDrive;
            double turn = maxDrivePower * (-gpButtons.robotTurn);
            double strafe = maxDrivePower * gpButtons.robotStrafe;

            chassis.drivingWithPID(drive, turn, strafe, true);

            // use Y button to lift up the slider reaching high junction
            if (gpButtons.sliderHighJunction) {
                slider.setInchPosition(Params.HIGH_JUNCTION_POS_TELE);
            }

            // use B button to lift up the slider reaching medium junction
            if (gpButtons.sliderMediumJunction) {
                slider.setInchPosition(Params.MEDIUM_JUNCTION_POS_TELE);
            }

            // use A button to lift up the slider reaching low junction
            if (gpButtons.sliderLowJunction) {
                slider.setInchPosition(Params.LOW_JUNCTION_POS_TELE);
            }

            // use X button to move the slider for wall position
            if (gpButtons.sliderWallPosition) {
                slider.setInchPosition(Params.WALL_POSITION);
            }
            // use dpad left to get to the ground junction position
            if (gpButtons.sliderGroundJunction) {
                if (armClaw.getArmPosition() > armClaw.ARM_FLIP_CENTER) {
                    armClaw.armFlipCenter();
                }
                slider.setInchPosition(Params.GROUND_JUNCTION_POS);
            }

            if (gpButtons.sliderGround) {
                if (armClaw.getArmPosition() > armClaw.ARM_FLIP_CENTER) {
                    armClaw.armFlipCenter();
                }
                slider.setInchPosition(slider.SLIDER_MIN_POS);
            }

            // use right stick_Y to lift or down slider continuously
            if (Math.abs(gpButtons.sliderUpDown) > 0) {
                telemetry.addData("gamepad", "%.2f", gpButtons.sliderUpDown);
                if ((armClaw.getArmPosition() > armClaw.ARM_FLIP_CENTER) &&
                        (slider.getPosition() < Params.WALL_POSITION * slider.COUNTS_PER_INCH)) {
                    armClaw.armFlipCenter();
                }
                slider.manualControlPos(gpButtons.sliderUpDown);
            }

            // Set position only when button is hit.
            if (gpButtons.clawClose) {
                armClaw.clawClose();
            }

            // Set position only when button is hit.
            if (gpButtons.clawOpen) {
                armClaw.clawOpen();
            }

            // turn arm left
            if (gpButtons.armLeft) {
                armClaw.armSwingTurnLeft();
            }

            // turn arm right
            if (gpButtons.armRight) {
                armClaw.armSwingTurnRight();
            }

            // turn arm forward
            if (gpButtons.armForward) {
                armClaw.armSwingTurnForward();
            }

            if (gpButtons.armFrontLoad) {
                armClaw.armFlipFrontLoad();
            }

            if (gpButtons.armFrontUnload) {
                armClaw.armFlipFrontUnload();
            }

            if (gpButtons.armBackLoad) {
                if (slider.getPosition() < Params.WALL_POSITION * slider.COUNTS_PER_INCH) {
                    slider.setInchPosition(Params.WALL_POSITION);
                }
                armClaw.armFlipBackLoad();
            }

            if (gpButtons.armBackUnload) {
                if (slider.getPosition() < Params.WALL_POSITION * slider.COUNTS_PER_INCH) {
                    slider.setInchPosition(Params.WALL_POSITION);
                }
                armClaw.armFlipBackUnloadTele();
            }

            // 0.2 is to avoid pressing button by mistake.
            if(Math.abs(gpButtons.armManualControl) > 0.2) {
                if (slider.getPosition() < Params.WALL_POSITION * slider.COUNTS_PER_INCH) {
                    slider.setInchPosition(Params.WALL_POSITION);
                }
                armClaw.armManualMoving(gpButtons.armManualControl);
            }

            //  auto driving, grip cone, and lift slider
            if (gpButtons.autoLoadGroundCone) {
                loadCone(Params.GROUND_CONE_POSITION); // Always on ground during teleop mode
            }

            //  auto driving, grip cone, and lift slider
            if (gpButtons.autoLoad34thConeStack) {
                loadCone(Params.GROUND_CONE_POSITION + Params.coneLoadStackGap * 2); // Always on ground during teleop mode
            }

            //  auto driving, grip cone, and lift slider
            if (gpButtons.autoLoad45thConeStack) {
                loadCone(Params.GROUND_CONE_POSITION + Params.coneLoadStackGap * 3); // Always on ground during teleop mode
            }

            //  auto driving, grip cone, and lift slider
            if (gpButtons.autoLoadThenJunction) {
                loadConeThenDriving(); // Always on ground during teleop mode for special pickup
            }

            //  auto driving, unload cone
            if (gpButtons.autoUnloadCone) {
                unloadCone(moveOutJunctionDistance);
            }

            // loading cone then moving to high junction
            if (gpButtons.autoUnloadThenBase) {
                unloadConeThenDriving();
            }

            // auto drop off cone, moving to cone base, auto pick up cone, then moving to junction.
            if (gpButtons.teapot) {
                unloadConeThenDriving();
                slider.waitRunningComplete();
                loadConeThenDriving();
            }

            if (debugFlag) {
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

                Logging.log("Get drive power = %.2f, set drive power = %.2f", chassisCurrentPower, maxP);
                Logging.log("Ctrl hub current = %.2f, max = %.2f", ctrlHubCurrent, maxCtrlCurrent);
                Logging.log("Ctrl hub volt = %.2f, min = %.2f", ctrlHubVolt, minCtrlVolt);
                Logging.log("Extend hub current = %.2f, max = %.2f", exHubCurrent, maxExCurrent);
                Logging.log("Extend hub volt = %.2f, min = %.2f", exHubVolt, minExVolt);
                Logging.log("Auxiliary voltage = %.2f, min = %.2f", auVolt, minAuVolt);

                // imu log
                chassis.getAngle(); // update last angles and global angle before logging
                telemetry.addData("imu heading yaw ", "%.2f", chassis.lastAngles.getYaw(AngleUnit.DEGREES));
                telemetry.addData("global heading ", "%.2f", chassis.globalAngle);
                telemetry.addData("Correction  ", "%.2f", chassis.correction);

                // claw arm servo log
                telemetry.addData("Arm", "position = %.2f", armClaw.getArmPosition());
                telemetry.addData("Claw", "position %.2f", armClaw.getClawPosition());

                telemetry.addData("Right slider", "current position %d",
                        slider.RightSliderMotor.getCurrentPosition());
                telemetry.addData("Left slider", "current position %d",
                        slider.LeftSliderMotor.getCurrentPosition());

                // drive motors log
                telemetry.addData("Max driving power ", "%.2f", maxDrivePower);
            }

            // running time
            telemetry.addData("Status", "While loop Time in ms = ", "%.1f", deltaTime);
            telemetry.update(); // update message at the end of while loop
            Logging.log("While loop time in ms = %.1f.", deltaTime);
        }

        // The motor stop on their own but power is still applied. Turn off motor.
        slider.stop();
        chassis.setPowers(0.0);
    }

    /**
     * During autonomous, cone may be located with different height position
     * @param coneLocation the target cone high location in inch.
     */
    private void loadCone(double coneLocation) {
        armClaw.armFlipFrontLoad();
        armClaw.clawOpen();
        slider.setInchPosition(coneLocation);
        chassis.runToPosition(-Params.DISTANCE_PICK_UP, true); // moving to loading position
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(Params.CLAW_CLOSE_SLEEP); // wait to make sure clawServo is at grep position, 200 ms
        slider.setInchPosition(Params.LOW_JUNCTION_POS_TELE);
        armClaw.armFlipBackUnloadTele();
    }

    /**
     * Special using case for loading cone from base, then driving to nearest high junction
     */
    private void loadConeThenDriving() {
        armClaw.armFlipFrontLoad();
        armClaw.clawOpen();
        slider.setInchPosition(Params.GROUND_CONE_POSITION);
        chassis.runToPosition(-Params.DISTANCE_PICK_UP, true); // moving to loading position
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(Params.CLAW_CLOSE_SLEEP); // 200 ms
        slider.setInchPosition(Params.HIGH_JUNCTION_POS_TELE);
        armClaw.armFlipCenter();
        chassis.runToPosition(-Params.BASE_TO_JUNCTION, true);
        slider.waitRunningComplete();
        armClaw.armFlipBackUnloadTele();
    }

    /**
     *
     * @param drivingDistance the driving back distance after unloading the cone.
     */
    private void unloadCone(double drivingDistance) {
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION);
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // to make sure claw Servo is at open position
        armClaw.armFlipFrontLoad();
        chassis.runToPosition(drivingDistance, true); // move out from junction
        slider.setInchPosition(Params.WALL_POSITION);
    }

    /**
     * Special using case for unloading cone on high junction, then driving to cone base
     */
    private void unloadConeThenDriving() {
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION);
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // to make sure claw Servo is at open position

        armClaw.armFlipFrontLoad();

        // driving back to cone base
        chassis.runToPosition(moveOutJunctionDistance, true); // move out from junction

        slider.setInchPosition(Params.WALL_POSITION - Params.coneLoadStackGap * 3);

        // -2 ~ 2 inch adjust moving to base for pick up
        chassis.runToPosition(Params.BASE_TO_JUNCTION - moveOutJunctionDistance, true);
    }
}
