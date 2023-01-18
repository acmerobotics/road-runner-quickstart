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

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.drive.strafeSpeed;
import static org.firstinspires.ftc.teamcode.constants.drive.turnSpeed;
import static org.firstinspires.ftc.teamcode.constants.slides.slideOnePID;
import static org.firstinspires.ftc.teamcode.constants.slides.slidePosArray;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.commands.servoCommand;
import org.firstinspires.ftc.teamcode.constants;

@TeleOp(name = "mecanumReal", group = "Competition")
// @Disabled
public class mecanumReal extends LinearOpMode {

  int slideOneTicks = 0;
  double strafeSpeed = constants.drive.strafeSpeed;
  double turnSpeed = constants.drive.turnSpeed;


  public static Orientation angles;
  public static Acceleration gravity;

  //button press
  private boolean isX = false;
  private boolean isY = false;
  private boolean isA = false;
  private boolean isB = false;
  private boolean isUp = false;
  private boolean isDown = false;
  private boolean speedIsDown = false;
  private boolean speedIsUp = false;
  private boolean turnSpeedIsDown;
  private boolean turnSpeedIsUp;

  //button press stuff
  private boolean wasX = false;
  private boolean wasY = false;
  private boolean wasA = false;
  private boolean wasB = false;
  private boolean wasUp = false;
  private boolean wasDown = false;
  private boolean speedWasUp = false;
  private boolean speedWasDown = false;
  private boolean turnSpeedWasUp;
  private boolean turnSpeedWasDown;



  @Override
  public void runOpMode() throws InterruptedException {
    // Declare OpMode members.

    DcMotor lF = hardwareMap.dcMotor.get("front_left");
    DcMotor lB = hardwareMap.dcMotor.get("back_left");
    DcMotor rF = hardwareMap.dcMotor.get("front_right");
    DcMotor rB = hardwareMap.dcMotor.get("back_right");
    DcMotor slideOne = hardwareMap.dcMotor.get("slide_one");
    Servo clawLeft = hardwareMap.servo.get("claw_left");
    Servo clawRight = hardwareMap.servo.get("claw_right");

    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    imu.initialize(parameters);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Set motor directions
    lF.setDirection(DcMotor.Direction.REVERSE);
    rF.setDirection(DcMotor.Direction.FORWARD);
    lB.setDirection(DcMotor.Direction.REVERSE);
    rB.setDirection(DcMotor.Direction.FORWARD);
    slideOne.setDirection(DcMotor.Direction.REVERSE);

    // Set zero power behavior
    lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //servo setting
    clawLeft.setDirection(Servo.Direction.REVERSE);
    clawRight.setDirection(Servo.Direction.FORWARD);


    //reset encoders
    slideOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideOne.setTargetPosition(slidePosArray[0]);
    //set encoder behavior
    slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //variables
    int slidePos = 0;

    PIDController slideOneController = new PIDController(slideOnePID[0], slideOnePID[1], slideOnePID[2], false);

    //set default target position
    slideOne.setTargetPosition(slidePosArray[0]);

    servoCommand.clawClose(clawLeft, clawRight);

    // Wait for the game to start (driver presses PLAY)
    waitForStart();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

        //ground
        if ((isA = gamepad1.a) && !wasA) {
          slideOne.setTargetPosition(slidePosArray[0]);
        }
        //high
        else if ((isB = gamepad1.b) && !wasB) {
          slideOne.setTargetPosition(slidePosArray[3]);
        }
        //medium
        else if ((isY = gamepad1.y) && !wasY) {
          slideOne.setTargetPosition(slidePosArray[2]);
        }
        //low
        else if ((isX = gamepad1.x) && !wasX) {
          slideOne.setTargetPosition(slidePosArray[1]);
        }

        //manual slide control
        if ((isUp = gamepad1.dpad_up) && !wasUp) {

            slideOne.setTargetPosition(slideOne.getTargetPosition() + 130);
          } else if ((isDown = gamepad1.dpad_down) && !wasDown) {
            slideOne.setTargetPosition(slideOne.getTargetPosition() - 130);
          }

        //slide limiter
        if (slideOne.getTargetPosition() > slidePosArray[3]) {
          slideOne.setTargetPosition(slidePosArray[3]);
        } else if (slideOne.getTargetPosition() < slidePosArray[0]) {
          slideOne.setTargetPosition(slidePosArray[0]);
        }


      // call "update" method and prepare motorPower
      double slideOnePower = slideOneController.update(
        slideOne.getTargetPosition(),
        slideOne.getCurrentPosition()
      );

      //assign motor the Pid output
      slideOne.setPower(slideOnePower);

      if(gamepad1.right_trigger == 1){
        servoCommand.clawClose(clawLeft, clawRight);
      } else if(gamepad1.left_trigger == 1){
        servoCommand.clawOpen(clawLeft, clawRight);
      }

      double y = -gamepad1.left_stick_y * strafeSpeed; // Remember, this is reversed!
      double x = gamepad1.left_stick_x * 1.1 * strafeSpeed; // Counteract imperfect strafing
      double rx = gamepad1.right_stick_x * turnSpeed;

      // Read inverse IMU heading, as the IMU heading is CW positive
      double botHeading = -imu.getAngularOrientation().firstAngle;

      double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
      double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio, but only when
      // at least one is out of the range [-1, 1]
      double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      // Send calculated power to motors
      if (gamepad1.left_bumper) {
        lF.setPower(frontLeftPower * 0.25);
        rF.setPower(frontRightPower * 0.25);
        lB.setPower(backLeftPower * 0.25);
        rB.setPower(backRightPower * 0.25);
      } else {
        lF.setPower(frontLeftPower);
        rF.setPower(frontRightPower);
        lB.setPower(backLeftPower);
        rB.setPower(backRightPower);
      }

      if((speedIsDown = gamepad2.dpad_down) && !speedWasDown){
        strafeSpeed -= 0.1;
      } else if((speedIsUp = gamepad2.dpad_up) && !speedWasUp){
        strafeSpeed += 0.1;
      }

      if((turnSpeedIsDown = gamepad2.dpad_right) && !turnSpeedWasDown){
        turnSpeed -= 0.1;
      } else if((turnSpeedIsUp = gamepad2.dpad_left) && !turnSpeedWasUp){
        turnSpeed += 0.1;
      }


      // reinitialize field oriented
      if (gamepad1.right_bumper) {
        imu.initialize(parameters);
      }

      //one button press does one thing
      wasA = isA;
      wasX = isX;
      wasY = isY;
      wasB = isB;
      wasUp = isUp;
      wasDown = isDown;
      speedWasUp = speedIsUp;
      speedWasDown = speedIsDown;
      turnSpeedWasDown = turnSpeedIsDown;
      turnSpeedWasUp = turnSpeedIsUp;



      telemetry.addData("Strafe Speed: ", strafeSpeed);
      telemetry.addData("Turn Speed:", turnSpeed);
      telemetry.addData("Heading: ", botHeading);
      telemetry.addData("Slide 1 Target (in):", slideOne.getTargetPosition() / 130);
      telemetry.addData("Slide 1 Height (in):", slideOne.getCurrentPosition() / 130);

      if(slideOne.getTargetPosition() == slidePosArray[0]){
        telemetry.addData("Slide 1 Position: ", "Ground");
      } else if(slideOne.getTargetPosition() == slidePosArray[1]){
        telemetry.addData("Slide 1 Position: ", "Low");
      } else if(slideOne.getTargetPosition() == slidePosArray[2]){
        telemetry.addData("Slide 1 Position: ", "Medium");
      } else if(slideOne.getTargetPosition() == slidePosArray[3]){
        telemetry.addData("Slide 1 Position: ", "High");
      }

      if(clawLeft.getPosition() == constants.servos.clawClosedLeft && clawRight.getPosition() == constants.servos.clawClosedRight){
        telemetry.addData("Claw Position: ", "Closed");
      } else if(clawLeft.getPosition() == constants.servos.clawOpenLeft && clawRight.getPosition() == constants.servos.clawOpenRight){
        telemetry.addData("Claw Position: ", "Open");
      }

      telemetry.update();
    }
  }
}
