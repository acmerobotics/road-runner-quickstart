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

package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.constants.slides.slideOnePID;
import static org.firstinspires.ftc.teamcode.constants.slides.slidePosArray;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name = "slideTestingPID", group = "Testing")
// @Disabled
public class slideTestingPID extends LinearOpMode {

    int slideOneTicks = 0;

    private boolean autoSlides = true;

    //button press
    private boolean isX = false;
    private boolean isY = false;
    private boolean isA = false;
    private boolean isB = false;
    private boolean isUp = false;
    private boolean isDown = false;

    //button press stuff
    private boolean wasX = false;
    private boolean wasY = false;
    private boolean wasA = false;
    private boolean wasB = false;
    private boolean wasUp = false;
    private boolean wasDown = false;



    @Override
    public void runOpMode() throws InterruptedException {
        // Declare OpMode members.

        DcMotor slideOne = hardwareMap.dcMotor.get("slide_one");



        // Set motor directions
        slideOne.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reset encoders
        slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //default target
        slideOne.setTargetPosition(0);

        //set encoder behavior
        slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //variables
        int slidePos = 0;

        PIDController slideOneController = new PIDController(slideOnePID[0], slideOnePID[1], slideOnePID[2], false);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //change autoSlides setting
            if (gamepad1.dpad_left) {
                autoSlides = true;
            } else if (gamepad1.dpad_right) {
                autoSlides = false;
            }

            //autoSlides on
            if (autoSlides) {
                //ground
                if ((isA = gamepad1.a) && !wasA) {
                    slidePos = 0;
                }
                //high
                else if ((isB = gamepad1.b) && !wasB) {
                    slidePos = 3;
                }
                //medium
                else if ((isY = gamepad1.y) && !wasY) {
                    slidePos = 2;
                }
                //low
                else if ((isX = gamepad1.x) && !wasX) {
                    slidePos = 1;
                }
                //sets target position to determined state
                slideOne.setTargetPosition(slidePosArray[slidePos]);
            }

            //auto slides off
            else {
                //manual slides up
                if (gamepad1.left_stick_y <= 0.7) {
                    slideOne.setTargetPosition(slideOne.getTargetPosition() + 1);
                }
                //manual slides down
                else if (gamepad1.left_stick_y <= 0.7) {
                    slideOne.setTargetPosition(slideOne.getTargetPosition() - 1);
                }
            }

            // call "update" method and prepare motorPower
            double slideOnePower = slideOneController.update(
                    slideOne.getTargetPosition(),
                    slideOne.getCurrentPosition()
            );


            // assign motor the PID output
            slideOne.setPower(slideOnePower);







            //one button press does one thing
            wasA = isA;
            wasX = isX;
            wasY = isY;
            wasB = isB;
            wasUp = isUp;
            wasDown = isDown;

            telemetry.addData("Slide 1 Ticks:", slideOneTicks);
            telemetry.addData("Slide 1 Power:", slideOnePower);
            telemetry.addData("Slide 1 Target:", slideOne.getTargetPosition());
            telemetry.addData("AutoSlides:", autoSlides);
            telemetry.update();
        }
    }
}
