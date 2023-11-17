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
 /*
  * PID controller and IMU codes are copied from
  * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
  */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    MecanumDrive mecanum;

    //claw and arm unit
    private intakeUnit intake;

    private Servo DroneServo;

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    private AprilTagTest tag = null;

    final double DESIRED_DISTANCE = 3.0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();


        mecanum = new MecanumDrive(hardwareMap, Params.currentPose);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tag = new AprilTagTest(mecanum, hardwareMap, 0, "Webcam 1");

        tag.initAprilTag();

        intake = new intakeUnit(hardwareMap, "ArmMotor", "WristServo", "FingerServo", "SwitchServo");

        intake.setArmModeRunToPosition(intake.getArmPosition());

        DroneServo = hardwareMap.get(Servo.class, "Drone");
        DroneServo.setPosition(0.5);

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start: %s", (Params.blueOrRed > 0)? "Blue" : "Red");

        telemetry.update();
        waitForStart();
        runtime.reset();

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
            mecanum.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * maxDrivePower,
                            -gpButtons.robotStrafe * maxDrivePower
                    ),
                    -gpButtons.robotTurn * maxDrivePower
            ));

            // Set position only when button is hit.
            if (gpButtons.wristDown) {
                intake.wristDown();
            }

            // Set position only when button is hit.
            if (gpButtons.wristUp) {
                intake.wristUp();
            }

            if (gpButtons.fingerOuttake) {
                intake.fingerOuttake();
            }

            if (gpButtons.fingerStop) {
                intake.fingerStop();
            }

            if (gpButtons.fingerIntake) {
                intake.fingerIntake();
            }

            if (!gpButtons.speedCtrl) {
                if (gpButtons.armUp) {
                    intake.armLiftAcc();
                }

                if (gpButtons.armDown) {
                    intake.armDownAcc();
                }
            }
            else {
                if (gpButtons.armUp) {
                    intake.armUp();
                }

                if (gpButtons.armDown) {
                    intake.armDown();
                }
            }

            if(gpButtons.readyToIntake) {
                intake.intakePositions(intake.ARM_POS_INTAKE);
            }
            if(gpButtons.readyToIntake2nd) {
                intake.intakePositions(intake.ARM_POS_INTAKE2);
            }
            if(gpButtons.readyToIntake3rd) {
                intake.intakePositions(intake.ARM_POS_INTAKE3);;
            }
            if(gpButtons.readyToIntake4th) {
                intake.intakePositions(intake.ARM_POS_INTAKE4);
            }

            if(gpButtons.switchOpen) {
                intake.switchServoOpen();
            }

            if(gpButtons.switchDropOne) {
                intake.switchServoDropOne();
            }

            if(gpButtons.switchClose) {
                intake.switchServoClose();
            }

            if(gpButtons.readyToDrop) {
                intake.dropPositions();
            }

            if (gpButtons.hangingRobot) {
                intake.hangingRobot();
            }

            if (gpButtons.droneLaunch) {
                DroneServo.setPosition(0);
            }

            if (gpButtons.moveToLeftTag) {
                moveByAprilTag(1 + ((Params.blueOrRed > 0)? 0 : 3));

            }

            if (gpButtons.moveToCenterTag) {
                moveByAprilTag(2 + ((Params.blueOrRed > 0)? 0 : 3));
            }

            if (gpButtons.moveToRightTag) {
                moveByAprilTag(3 + ((Params.blueOrRed > 0)? 0 : 3));
            }

            if (debugFlag) {
                // claw arm servo log
                telemetry.addData("Wrist", "position %.2f", intake.getWristPosition());

                telemetry.addData("Arm", "position = %d", intake.getArmPosition());

                telemetry.addData("Finger", "position %.2f", intake.getFingerPosition());

                telemetry.addData("switch", "position %.2f", intake.getSwitchPosition());


                telemetry.addData("Drone", "position %.2f", DroneServo.getPosition());

                telemetry.update(); // update message at the end of while loop

                Logging.log("Wrist position %.2f", intake.getWristPosition());

            }
        }

        intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The motor stop on their own but power is still applied. Turn off motor.
    }

    private void logVector(String sTag, Vector2d vXY) {
        String vectorName = vXY.toString();
        Logging.log("%s: %s", sTag, vectorName);
    }

    private void moveByAprilTag(int tagNum) {
        intake.dropPositions();
        sleep(300); // make sure arm is out of camera sight

        Pose2d aprilTagPose = tag.updatePoseAprilTag(tagNum);

        // if can not move based on April tag, moved by road runner.
        if (tag.targetFound) {
            mecanum.updatePoseEstimate();
            mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // turn
            Actions.runBlocking(
                    mecanum.actionBuilder(mecanum.pose)
                            .turn(aprilTagPose.heading.log())
                            .build());

            aprilTagPose = tag.updatePoseAprilTag(tagNum); // update position values after turn

            Logging.log("yaw = %.2f", aprilTagPose.heading.log());
            logVector("robot drive: distance from camera to april tag", aprilTagPose.position);
            if (tag.targetFound) {

                // adjust yellow drop-off position according to april tag location info from camera
                Vector2d desiredMove = new Vector2d(mecanum.pose.position.x - aprilTagPose.position.x,
                        mecanum.pose.position.y - aprilTagPose.position.y + Params.TELEOP_DISTANCE_TO_TAG);
                logVector("robot drive: move to tag distance", desiredMove);

                // shift to AprilTag
                Actions.runBlocking(
                        mecanum.actionBuilder(mecanum.pose)
                                .strafeTo(desiredMove)
                                .build()
                );
            }
            mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

}
