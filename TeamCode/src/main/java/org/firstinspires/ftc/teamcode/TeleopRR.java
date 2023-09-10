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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/**
 * Hardware config:
 *      imu on control Hub:
 *          "imu"
 *
 *      Four drive motors:
 *          "FrontLeft"
 *          "BackLeft"
 *          "BackRight"
 *          "FrontRight"
 *
 *      Tow slider motors:
 *          "RightSlider"
 *          "LeftSlider"
 *
 *      Tow servo motors:
 *          "ArmServo"
 *          "ClawServo"
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

    private Servo launchServo = null;

    // variables for auto load and unload cone
    double moveOutJunctionDistance = 5.0; // in INCH

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();

        mecanum = new SampleMecanumDrive(hardwareMap);


        mecanum.setPoseEstimate(Params.currentPose); // load the pose of the end of autonomous

        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armClaw.init(hardwareMap, "ArmMotor", "ClawServo");

        armClaw.resetArmEncoder();

        launchServo = hardwareMap.get(Servo.class, "LaunchServo");

        launchServo.setPosition(0.5);

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
            //slider.setInchPosition(Params.WALL_POSITION);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            double maxDrivePower;
            if (gpButtons.speedUp) {
                maxDrivePower = Params.POWER_HIGH;
            } else if (gpButtons.speedDown) {
                maxDrivePower = Params.POWER_LOW;
            } else {
                maxDrivePower = Params.POWER_NORMAL;
            }

            mecanum.setWeightedDrivePower(
                    new Pose2d(
                            gpButtons.robotDrive * maxDrivePower,
                            gpButtons.robotStrafe * maxDrivePower,
                            -gpButtons.robotTurn * maxDrivePower
                    )
            );


            // Set position only when button is hit.
            if (gpButtons.clawClose) {
                armClaw.clawClose();
            }

            // Set position only when button is hit.
            if (gpButtons.clawOpen) {
                armClaw.clawOpen();
            }


            if(Math.abs(gpButtons.armManualControl) > 0) {
                armClaw.armManualMoving(gpButtons.armManualControl);
            }

            if (gpButtons.launchOn) {
                launchServo.setPosition(0);
            }

            if (gpButtons.armLift) {
                armClaw.armLift();
            }

            if (gpButtons.armDown) {
                armClaw.armDown();
            }


            if (debugFlag) {
                // claw arm servo log
                telemetry.addData("Claw", "position %.2f", armClaw.getClawPosition());

                telemetry.addData("RR", "imu Heading = %.1f",
                        Math.toDegrees(mecanum.getRawExternalHeading()));

                mecanum.updatePoseEstimate();
                telemetry.addData("RR", "x = %.1f, y = %.1f, Heading = %.1f",
                        mecanum.getPoseEstimate().getX(), mecanum.getPoseEstimate().getY(),
                        Math.toDegrees(mecanum.getPoseEstimate().getHeading()));

                telemetry.addData("Arm", "position = %.2f", armClaw.getArmPosition());

                telemetry.addData("Claw", "position %.2f", armClaw.getClawPosition());

                telemetry.update(); // update message at the end of while loop
            }
        }

        // The motor stop on their own but power is still applied. Turn off motor.
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
        driveBack(Params.DISTANCE_PICK_UP); // moving to loading position
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(Params.CLAW_CLOSE_SLEEP); // wait to make sure clawServo is at grep position, 200 ms
        slider.setInchPosition(Params.LOW_JUNCTION_POS);
        armClaw.armFlipBackUnloadPre();
    }

    /**
     * Special using case for loading cone from base, then driving to nearest high junction
     */
    private void loadConeThenDriving() {
        armClaw.armFlipFrontLoad();
        armClaw.clawOpen();
        slider.setInchPosition(Params.GROUND_CONE_POSITION);
        driveBack(Params.DISTANCE_PICK_UP); // moving to loading position
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(Params.CLAW_CLOSE_SLEEP);
        slider.setInchPosition(Params.HIGH_JUNCTION_POS);
        armClaw.armFlipCenter();
        driveBack(Params.BASE_TO_JUNCTION - Params.DISTANCE_PICK_UP);
        slider.waitRunningComplete();
        armClaw.armFlipBackUnloadPre();
    }

    /**
     *
     * @param drivingDistance the driving back distance after unloading the cone.
     */
    private void unloadCone(double drivingDistance) {
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION);
        slider.waitRunningComplete();
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // to make sure claw Servo is at open position
        armClaw.armFlipFrontLoad();
        driveForward(drivingDistance); // move out from junction
        sleep(100);
        slider.setInchPosition(Params.GROUND_CONE_READY_POSITION);
    }

    /**
     * Special using case for unloading cone on high junction, then driving to cone base
     */
    private void unloadConeThenDriving() {
        mecanum.setPoseEstimate(new Pose2d());
        mecanum.setMode(DriveConstants.with3DW? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER);
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION);
        slider.waitRunningComplete();
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // to make sure claw Servo is at open position

        armClaw.armFlipFrontLoad();

        //combines drive back and lower slider action
        Trajectory unloadTraj = mecanum.trajectoryBuilder(mecanum.getPoseEstimate())
                .forward(Params.BASE_TO_JUNCTION)
                .addDisplacementMarker(moveOutJunctionDistance, () -> {
                    slider.setInchPosition(Params.GROUND_CONE_READY_POSITION);
                })
                .build();
        mecanum.followTrajectory(unloadTraj);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void driveBack(double distanceInch) {
        mecanum.setPoseEstimate(new Pose2d());
        mecanum.setMode(DriveConstants.with3DW? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER);
        Trajectory trajBack = mecanum.trajectoryBuilder(mecanum.getPoseEstimate())
                .back(distanceInch)
                .build();
        mecanum.followTrajectory(trajBack);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void driveForward(double distanceInch) {
        mecanum.setPoseEstimate(new Pose2d());
        mecanum.setMode(DriveConstants.with3DW? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER);
        Trajectory trajBack = mecanum.trajectoryBuilder(mecanum.getPoseEstimate())
                .forward(distanceInch)
                .build();
        mecanum.followTrajectory(trajBack);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
