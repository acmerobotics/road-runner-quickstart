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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.constants.servos.clawClosedLeft;
import static org.firstinspires.ftc.teamcode.constants.servos.clawClosedRight;
import static org.firstinspires.ftc.teamcode.constants.servos.clawOpenLeft;
import static org.firstinspires.ftc.teamcode.constants.servos.clawOpenRight;
import static org.firstinspires.ftc.teamcode.constants.slides.slidePosArray;
import static org.firstinspires.ftc.teamcode.constants.slides.slideOnePID;
import static org.firstinspires.ftc.teamcode.constants.drive.driveSpeed;
import static org.firstinspires.ftc.teamcode.constants.slides.slideTwoPID;
import static org.firstinspires.ftc.teamcode.constants.motors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Parameter;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.commands.servoCommand;

@TeleOp(name = "mecanumReal", group = "Competition")
// @Disabled
public class mecanumReal extends LinearOpMode {

  int slideOneTicks = 0;
  double speed = 0.7;


  public static Orientation angles;
  public static Acceleration gravity;

  private boolean autoSlides = true;

  //button press
  private boolean isX = false;
  private boolean isY = false;
  private boolean isA = false;
  private boolean isB = false;
  private boolean isUp = false;
  private boolean isDown = false;
  private boolean speedIsDown = false;
  private boolean speedIsUp = false;

  //button press stuff
  private boolean wasX = false;
  private boolean wasY = false;
  private boolean wasA = false;
  private boolean wasB = false;
  private boolean wasUp = false;
  private boolean wasDown = false;
  private boolean speedWasUp = false;
  private boolean speedWasDown = false;



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

        //manual slides up
        if ((isUp = gamepad1.dpad_up) && !wasUp) {
          if (slideOne.getTargetPosition() >= 4355) {
            slideOne.setTargetPosition(4355);
          }
          else {
            slideOne.setTargetPosition(slideOne.getTargetPosition() + 130);
          }
        }
        //manual slides down
        else if ((isDown = gamepad1.dpad_down) && !wasDown) {
          if (slideOne.getTargetPosition() <= 0) {
            slideOne.setTargetPosition(0);
          }
          else {
            slideOne.setTargetPosition(slideOne.getTargetPosition() - 130);
          }
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



      // Setup a variable for each drive wheel to save power level for telemetry
      double frontLeftPower;
      double frontRightPower;
      double backLeftPower;
      double backRightPower;

      // set gamepad values
      double x = gamepad1.left_stick_x * 1.1;
      double y = -gamepad1.left_stick_y;
      double t = gamepad1.right_stick_x;
      double botHeading = -imu.getAngularOrientation().firstAngle;

      // rotation
      double x_rotated =
        x * Math.cos(botHeading) - y * Math.sin(botHeading);
      double y_rotated =
        x * Math.sin(botHeading) + y * Math.cos(botHeading);

      // x, y, theta input mixing
      double denominator = Math.max(Math.abs(y_rotated) + Math.abs(x_rotated) + Math.abs(t), 1);
      frontLeftPower = (y + x + t) / denominator;
      backLeftPower = (y - x + t) / denominator;
      frontRightPower = (y - x - t) / denominator;
      backRightPower = (y + x - t) / denominator;

      // Send calculated power to motors
      if (gamepad1.left_bumper) {
        lF.setPower(frontLeftPower * speed * 0.25);
        rF.setPower(frontRightPower * speed * 0.25);
        lB.setPower(backLeftPower * speed * 0.25);
        rB.setPower(backRightPower * speed * 0.25);
      } else {
        lF.setPower(frontLeftPower * speed);
        rF.setPower(frontRightPower * speed);
        lB.setPower(backLeftPower * speed);
        rB.setPower(backRightPower * speed);
      }

      if((speedIsDown = gamepad2.dpad_down) && !speedWasDown){
        speed -= 0.1;
      } else if((speedIsUp = gamepad2.dpad_up) && !speedWasUp){
        speed += 0.1;
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



      telemetry.addData("Speed: ", speed);
      telemetry.addData("Heading: ", botHeading);
      telemetry.addData("Slide One Position:", slideOne.getCurrentPosition());
      telemetry.addData("Slide 1 Target:", slideOne.getTargetPosition());
      telemetry.addData("Slide 1 Height (in):", slideOne.getCurrentPosition() / 130);
      telemetry.addData("AutoSlides:", autoSlides);
      telemetry.update();
    }
  }
}
