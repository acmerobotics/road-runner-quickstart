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

package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.autoCommands.*;
import static org.firstinspires.ftc.teamcode.constants.slides.slideOnePID;
import static org.firstinspires.ftc.teamcode.constants.slides.slidePosArray;



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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.autonomous.autoCommands;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.servoCommand;



@Autonomous(name="coneDetection", group="Auto", preselectTeleOp = "mecanumReal")
public class coneDetection extends LinearOpMode {

    //    /* Declare OpMode members. */


    private ElapsedTime runtime = new ElapsedTime();






    @Override
    public void runOpMode() {
        DcMotor lF = hardwareMap.dcMotor.get("front_left");
        DcMotor lB = hardwareMap.dcMotor.get("back_left");
        DcMotor rF = hardwareMap.dcMotor.get("front_right");
        DcMotor rB = hardwareMap.dcMotor.get("back_right");
        DcMotor slideOne = hardwareMap.dcMotor.get("slide_one");
        Servo clawLeft = hardwareMap.servo.get("claw_left");
        Servo clawRight = hardwareMap.servo.get("claw_right");
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
//        // Set motor directions
        lF.setDirection(DcMotor.Direction.FORWARD);
        rF.setDirection(DcMotor.Direction.REVERSE);
        lB.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.REVERSE);
        slideOne.setDirection(DcMotor.Direction.REVERSE);
//
//        // Set zero power behavior
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
        slideOne.setTargetPosition(0);
        //set encoder behavior
        slideOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //variables
        int slidePos = 0;

        PIDController slideOneController = new PIDController(slideOnePID[0], slideOnePID[1], slideOnePID[2], false);

        servoCommand.clawOpen(clawLeft, clawRight);

        sleep(5000);

        servoCommand.clawClose(clawLeft, clawRight);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//
       autoCommands.driveForward(lF, lB, rF, rB, 0.5);
        sleep(500);
        autoCommands.stop(lF, lB, rF, rB);

if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
    telemetry.addData("Color Red: ", colorSensor.red());
    telemetry.update();
    autoCommands.driveForward(lF, lB, rF, rB, 0.4);
    sleep(600);
    autoCommands.strafeLeft(lF, lB, rF, rB, 0.4);
    sleep(1700);
    autoCommands.stop(lF, lB, rF, rB);
}else if(colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()){
    telemetry.addData("Color Green: ", colorSensor.green());
    telemetry.update();
    autoCommands.driveForward(lF, lB, rF, rB, 0.4);
    sleep(600);
    autoCommands.stop(lF, lB, rF, rB);
} else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
    telemetry.addData("Color Blue: ", colorSensor.blue());
    telemetry.update();
    autoCommands.driveForward(lF, lB, rF, rB, 0.4);
    sleep(600);
    autoCommands.strafeRight(lF, lB, rF, rB, 0.4);
    sleep(1700);
    autoCommands.stop(lF, lB, rF, rB);
}
        autoCommands.stop(lF, lB, rF, rB);
servoCommand.clawClose(clawLeft, clawRight);
        telemetry.update();

    }
}
