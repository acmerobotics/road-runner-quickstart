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

package org.firstinspires.ftc.teamcode;

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
 * This particular OpMode illustrates driving a 2-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               A button and B button: Forward/Backward
 *
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

 @TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
 //@Disabled
 class Error404_Actuator_Slides extends LinearOpMode {

     // Declare OpMode members for each of the 3 motors.
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotor SlideBig;
     private DcMotor SlideLittle;

     @Override
     public void runOpMode() {

         // Initialize the hardware variables. Note that the strings used here must correspond
         // to the names assigned during the robot configuration step on the DS or RC devices.
         SlideBig = hardwareMap.get(DcMotor.class, "slide_big_move");
         SlideLittle = hardwareMap.get(DcMotor.class, "slide_little_move");

         // ########################################################################################
         // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
         // ########################################################################################
         // Most robots need the motors on one side to be reversed to drive forward.
         // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
         // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
         // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
         // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
         // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
         // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
         SlideBig.setTargetPosition(0);
         SlideLittle.setTargetPosition(0);
         SlideLittle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         SlideBig.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         SlideBig.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         SlideLittle.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         int SlideLittle = 0;
         int SlideBig = 0;
         // Wait for the game to start (driver presses PLAY)
         telemetry.addData("Status", "Initialized");
         telemetry.update();

         waitForStart();
         runtime.reset();

         // run until the end of the match (driver presses STOP)
         while (opModeIsActive()) {
             if (gamepad1.a) {
                 SlideLittle = 537; //Slide goes to full length
             }
             else if (gamepad1.b) {
                 SlideLittle = 0;
             }
             if (gamepad1.x) {
                 SlideBig = 537;  //Ticks per revolution
             }
             else if (gamepad1.y) {
                 SlideBig = 0;
                 }
             }
         }
     }
