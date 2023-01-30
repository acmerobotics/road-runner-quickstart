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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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

@TeleOp(name="Teleop RR", group="Concept")
//@Disabled
public class TeleopRR extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    SampleMecanumDrive mecanum;

    // slider motor power variables
    private final SlidersWith2Motors slider = new SlidersWith2Motors();

    //claw and arm unit
    private final ArmClawUnit armClaw = new ArmClawUnit();

    // variables for auto load and unload cone
    double moveOutJunctionDistance = 5.0; // in INCH

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();

        mecanum = new SampleMecanumDrive(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slider.init(hardwareMap, "RightSlider", "LeftSlider");
        slider.setInchPosition(Params.GROUND_CONE_POSITION);

        armClaw.init(hardwareMap, "ArmServo", "ClawServo");

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

        // move slider to wall position just when starting.
        if (opModeIsActive()) {
            slider.setInchPosition(Params.WALL_POSITION);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            mecanum.setWeightedDrivePower(
                    new Pose2d(
                            gpButtons.robotDrive,
                            -gpButtons.robotTurn,
                            gpButtons.robotStrafe
                    )
            );

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

                // claw arm servo log
                telemetry.addData("Arm", "position = %.2f", armClaw.getArmPosition());
                telemetry.addData("Claw", "position %.2f", armClaw.getClawPosition());

                telemetry.addData("Right slider", "current position %d",
                        slider.RightSliderMotor.getCurrentPosition());
                telemetry.addData("Left slider", "current position %d",
                        slider.LeftSliderMotor.getCurrentPosition());
            }

            // running time
            telemetry.update(); // update message at the end of while loop
        }

        // The motor stop on their own but power is still applied. Turn off motor.
        slider.stop();
        mecanum.setMotorPowers(0.0,0.0,0.0,0.0);
    }

    /**
     * During autonomous, cone may be located with different height position
     * @param coneLocation the target cone high location in inch.
     */
    private void loadCone(double coneLocation) {
        armClaw.armFlipFrontLoad();
        armClaw.clawOpen();
        slider.setInchPosition(coneLocation);
        driveForwardBack(-Params.DISTANCE_PICK_UP); // moving to loading position
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
        driveForwardBack(-Params.DISTANCE_PICK_UP); // moving to loading position
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(Params.CLAW_CLOSE_SLEEP); // 200 ms
        slider.setInchPosition(Params.HIGH_JUNCTION_POS_TELE);
        armClaw.armFlipCenter();
        driveForwardBack(-Params.BASE_TO_JUNCTION);
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
        driveForwardBack(-drivingDistance); // move out from junction
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
        driveForwardBack(moveOutJunctionDistance); // move out from junction

        slider.setInchPosition(Params.WALL_POSITION - Params.coneLoadStackGap * 3);

        // -2 ~ 2 inch adjust moving to base for pick up
        driveForwardBack(Params.BASE_TO_JUNCTION - moveOutJunctionDistance);
    }

    private void driveForwardBack(double distanceInch) {
        Trajectory trajectory = mecanum.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(distanceInch, 0))
                .build();

        mecanum.followTrajectory(trajectory);
    }

}
