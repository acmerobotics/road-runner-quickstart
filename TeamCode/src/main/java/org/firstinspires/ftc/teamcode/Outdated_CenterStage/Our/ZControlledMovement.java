/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */
//John - starts program and creates variables
@TeleOp(name="ControlledMovement", group="Linear OpMode")

@Disabled

public class ZControlledMovement extends LinearOpMode {

    // John - Starts a timer so you know how long the program has been running
    private ElapsedTime runtime = new ElapsedTime();

    // John - creates variables for all 4 motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double speedmult = 0.35;
    boolean Slowmode = true;
    double time1 = 0;
    @Override
    public void runOpMode() {

        //John - connects the variable to the actual motor. This makes the variable control the
        //       wheel movement. Make sure the device name is what you set them on the robot
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FRONT_L");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BACK_L");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FRONT_R");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BACK_R");

        //John - sets the direction the wheels move, change these to make the robot move the right way
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // John - Lets you know the code is ready to start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // John - Waits for you to press start button
        waitForStart();

        // John - starts over timer while the program is running
        runtime.reset();

        // John - main loop repeats until you press stop on the program
        while (opModeIsActive()) {

            // John - creates a variable that keeps track of decimal numbers
            double max;

            // John - creates variables for all directions each joystick moves.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value so y inputs should be negative
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Calculates speed of each wheel based on the inputs from the controller
            double leftFrontPower  = speedmult * (axial + lateral + yaw*(0.4064 + 0.3302));
            double rightFrontPower = speedmult * (axial - lateral - yaw*(0.4064 + 0.3302));
            double leftBackPower   = speedmult * (axial - lateral + yaw*(0.4064 + 0.3302));
            double rightBackPower  = speedmult * (axial + lateral - yaw*(0.4064 + 0.3302));

            // Calculates which wheel's speed is the fastest and makes sure that it isn't past that wheel's max speed
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > speedmult) {
                double conversion = speedmult / max;
                leftFrontPower = leftFrontPower * conversion;
                rightFrontPower = rightFrontPower * conversion;
                leftBackPower = leftBackPower * conversion;
                rightBackPower = rightBackPower * conversion;

            }

            double time2 = runtime.seconds();
            if (gamepad1.x && time2-time1 >= 1) {
                if (Slowmode) {
                    Slowmode = false;
                    speedmult = 1;
                } else {
                    Slowmode = true;
                    speedmult = 0.35;
                }
                time1 = runtime.seconds();
            }

            // John - if this is uncommented we can test each individual wheel by moving it with
            //        x, y, a, and b on the controller. If a button is pressed it will set wheel
            //        velocity to 1, if you don't it will set it to 0. Recomment when ready to
            //        test with joystick.
            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
          

            // John - moves wheels at calculated power
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // John - sends the elapsed time and wheel power to the computer so you can troubleshoot
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("yaw", yaw);
            telemetry.update();
        }

    }
}
