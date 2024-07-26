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
//other imports
import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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
@Autonomous(name="AutonomousMovementPID", group="Robot")

@Disabled

public class ZAutonomousMovementPID extends LinearOpMode {

    // John - Starts a timer so you know how long the program has been running
    ElapsedTime runtime = new ElapsedTime();

    //timer for time movement functions
    ElapsedTime runtimemotor = new ElapsedTime();

    //stuff for wait function
    ElapsedTime wait = new ElapsedTime();
    double time = 0;

    // John - creates variables for all 4 motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //wheel movement and turning stuff
    double angle = 0;
    double distance = 0;
    double speedmultiply = 0.35;
    IMU imu;

    //wheel stats
    double wheelmaxspeedrpm = 312;
    double wheelradius = 48;
    double wheelmaxspeedcmps = (((2 * Math.PI * wheelradius)/60) * wheelmaxspeedrpm)*100;
    double motorencoderres = 28;
    double gearboxres = 19.2;
    double wheelticksperrev = motorencoderres * gearboxres;

    //variables i needed for pid
    double error = 1;
    double proportionalmult;
    double integralmult;
    double derivativemult;
    double derivative;
    ElapsedTime timerLF = new ElapsedTime();
    ElapsedTime timerRF = new ElapsedTime();
    ElapsedTime timerLB = new ElapsedTime();
    ElapsedTime timerRB = new ElapsedTime();

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    //convert ticks/s to cmp/s (((2 * Math.PI * wheelradius)/60)*(tickspersec/wheelticksperrev/60))*100;
    @Override
    public void runOpMode() {
        initializewheels();
        initializeimu();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        angle = 0;
        distance = 20;
        movedistancetime();

    }
    private void waiterfor() {
        wait.reset();
        while (wait.seconds() < time){
            telemetry.addData("Status", "waiting");
        }
    }
    private void pidallwheels() {
        if (opModeIsActive()) {
            proportionalmult = .1;
            integralmult = .1;
            derivativemult = .1;
            double encoderLFpast = 0;
            double encoderRFpast = 0;
            double encoderLBpast = 0;
            double encoderRBpast = 0;
            double lasterrorLF = 0;
            double lasterrorRF = 0;
            double lasterrorLB = 0;
            double lasterrorRB = 0;
            double integralsumLF = 0;
            double integralsumRF = 0;
            double integralsumLB = 0;
            double integralsumRB = 0;
            double LFdistance = 0;
            double RFdistance = 0;
            double LBdistance = 0;
            double RBdistance = 0;
            timerLF.reset();
            timerRF.reset();
            timerLB.reset();
            timerRB.reset();
            runtimemotor.reset();
            while ((opModeIsActive()) && (runtimemotor.seconds() * speedmultiply * wheelmaxspeedcmps / 100 < distance)) {

                //calculates how far the wheel moved
                LFdistance = LFdistance + (((2 * Math.PI * wheelradius) * ((leftFrontDrive.getCurrentPosition() - encoderLFpast) / wheelticksperrev)) * 100);
                //does pid
                double tickspersecLF = (leftFrontDrive.getCurrentPosition() - encoderLFpast) / timerLF.seconds();
                double currentLFcmps = (((2 * Math.PI * wheelradius) / 60) * (tickspersecLF / wheelticksperrev / 60)) * 100;
                double errorLF = leftFrontPower * wheelmaxspeedcmps - currentLFcmps;
                derivative = (errorLF - lasterrorLF) / timerLF.seconds();
                integralsumLF = integralsumLF + (errorLF * timerLF.seconds());
                double outLF = (proportionalmult * errorLF) + (integralmult * integralsumLF) + (derivativemult * derivative);
                lasterrorLF = errorLF;
                encoderLFpast = leftFrontDrive.getCurrentPosition();
                timerLF.reset();

                RFdistance = RFdistance + (((2 * Math.PI * wheelradius) * ((rightFrontDrive.getCurrentPosition() - encoderRFpast) / wheelticksperrev)) * 100);
                double tickspersecRF = (rightFrontDrive.getCurrentPosition() - encoderRFpast) / timerRF.seconds();
                double currentRFcmps = (((2 * Math.PI * wheelradius) / 60) * (tickspersecRF / wheelticksperrev / 60)) * 100;
                double errorRF = rightFrontPower * wheelmaxspeedcmps - currentRFcmps;
                derivative = (errorRF - lasterrorRF) / timerRF.seconds();
                integralsumRF = integralsumRF + (errorRF * timerRF.seconds());
                double outRF = (proportionalmult * errorRF) + (integralmult * integralsumRF) + (derivativemult * derivative);
                lasterrorRF = errorRF;
                encoderRFpast = rightFrontDrive.getCurrentPosition();
                timerRF.reset();

                LBdistance = LBdistance + (((2 * Math.PI * wheelradius) * ((leftBackDrive.getCurrentPosition() - encoderLBpast) / wheelticksperrev)) * 100);
                double tickspersecLB = (leftBackDrive.getCurrentPosition() - encoderLBpast) / timerLB.seconds();
                double currentLBcmps = (((2 * Math.PI * wheelradius) / 60) * (tickspersecLB / wheelticksperrev / 60)) * 100;
                double errorLB = leftBackPower * wheelmaxspeedcmps - currentLBcmps;
                derivative = (errorLB - lasterrorLB) / timerLB.seconds();
                integralsumLB = integralsumLB + (errorLB * timerLB.seconds());
                double outLB = (proportionalmult * errorLB) + (integralmult * integralsumLB) + (derivativemult * derivative);
                lasterrorLB = errorLB;
                encoderLBpast = leftBackDrive.getCurrentPosition();
                timerLB.reset();

                RBdistance = RBdistance + (((2 * Math.PI * wheelradius) * ((rightBackDrive.getCurrentPosition() - encoderRBpast) / wheelticksperrev)) * 100);
                double tickspersecRB = (rightBackDrive.getCurrentPosition() - encoderRBpast) / timerRB.seconds();
                double currentRBcmps = (((2 * Math.PI * wheelradius) / 60) * (tickspersecRB / wheelticksperrev / 60)) * 100;
                double errorRB = rightBackPower * wheelmaxspeedcmps - currentRBcmps;
                derivative = (errorRB - lasterrorRB) / timerRB.seconds();
                integralsumRB = integralsumRB + (errorRB * timerRB.seconds());
                double outRB = (proportionalmult * errorRB) + (integralmult * integralsumRB) + (derivativemult * derivative);
                lasterrorRB = errorRB;
                encoderRBpast = rightBackDrive.getCurrentPosition();
                timerRB.reset();

                leftFrontDrive.setPower(leftFrontPower + outLF);
                rightFrontDrive.setPower(rightFrontPower + outRF);
                leftBackDrive.setPower(leftBackPower + outLB);
                rightBackDrive.setPower(rightBackPower + outRB);
                telemetry.addData("number", (((2 * Math.PI * wheelradius) / 60) * (tickspersecRB / wheelticksperrev / 60)) * 100);
                telemetry.addData("Left Front Output", outLF);
                telemetry.addData("Right Front Output", outRF);
                telemetry.addData("Left Back Output", outLB);
                telemetry.addData("Right Back Output", outRB);
                telemetry.addData("distance traveled", (LFdistance + RFdistance + LBdistance + RBdistance) / 4);
                telemetry.addData("LFError", errorLF);
                telemetry.addData("RFError", errorRF);
                telemetry.addData("LBError", errorLB);
                telemetry.addData("RBError", errorRB);
                telemetry.addData("currentposition", leftFrontDrive.getCurrentPosition());
                telemetry.update();
            }
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            telemetry.addData("isitdone", "yes");
        }
    }
    private void movedistancetime () {
        if (opModeIsActive()) {
            double max;
            double axial = 90 - abs(angle);
            double lateral = 90 - abs(angle - 90);
            leftFrontPower = (axial + lateral);
            rightFrontPower = (axial - lateral);
            leftBackPower = (axial - lateral );
            rightBackPower = (axial + lateral);
            //does max based on your speed mult
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            double conversion = speedmultiply/max;
            leftFrontPower = leftFrontPower * conversion;
            rightFrontPower = rightFrontPower * conversion;
            leftBackPower = leftBackPower * conversion;
            rightBackPower = rightBackPower * conversion;
            pidallwheels();
        }
    }
    private void turnanglegyro () {
        if(opModeIsActive()){
            double previoustime = 0;
            double turnspeed = 0;
            double integral = 0;
            error = 1;
            imu.resetYaw();
            runtimemotor.reset();
            while ((opModeIsActive()) && (abs(error) >= 0.1)){
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                error = (angle+orientation.getYaw(AngleUnit.DEGREES));
                integral = integral + ((error/70)*(runtimemotor.seconds() - previoustime));
                turnspeed = (speedmultiply * (error/70 + (integral * integralmult)));
                leftFrontPower = turnspeed;
                rightFrontPower = -turnspeed;
                leftBackPower = turnspeed;
                rightBackPower = -turnspeed;

                leftFrontDrive.setPower(leftFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightFrontDrive.setPower(rightFrontPower);
                rightBackDrive.setPower(rightBackPower);
                telemetry.addData("error", error);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
                previoustime = runtimemotor.seconds();
            }
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            telemetry.addData("Status", "done");
            telemetry.update();
        }
    }
    private void initializewheels() {
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

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void initializeimu() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

}