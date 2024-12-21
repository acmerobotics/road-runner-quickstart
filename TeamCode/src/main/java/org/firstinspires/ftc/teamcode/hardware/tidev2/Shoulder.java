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

package org.firstinspires.ftc.teamcode.hardware.tidev2;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shoulder {
    public final double MAX_POWER = 2.0;
    public final double MIN_POWER = -0.5;



    static final double deadzone = 0.3;


    private PIDFController controller;

    public static final double p = 0.003, i = 0.1, d = 0.0002;
    public static final double f = 0.00003;

    public static int target = 100;

    double pidf;


    private OpMode myOpMode;   // gain access to methods in the calling OpMode.


    private DcMotorEx shoulder_right;
    private DcMotorEx shoulder_left;

    public Shoulder(OpMode opmode) {
        myOpMode = opmode;
    }

    int armPos;


    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(50, 100);

        shoulder_right = myOpMode.hardwareMap.get(DcMotorEx.class, "left_tower");
        shoulder_left = myOpMode.hardwareMap.get(DcMotorEx.class, "right_tower");

        shoulder_right.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder_left.setDirection(DcMotorSimple.Direction.FORWARD);

        shoulder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulder_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidf = 0;
        target = 0;

    }



    public class AutonListen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            autoListen();
            return true;
        }
    }
    public Action autonListen() {
        return new AutonListen();
    }


    public class AutonHC implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setTarget(540);
            return false;
        }
    }
    public Action autonHC() {
        return new AutonHC();
    }

    public class AutonDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setTarget(70);
            return false;
        }
    }
    public Action autonDown() {
        return new AutonDown();
    }

    public class AutonMidDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setTarget(200);
            return false;
        }
    }
    public Action autonMidDown() {
        return new AutonMidDown();
    }

    public class AutonDownHC implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setTarget(400);
            return false;
        }
    }
    public Action autonDownHC() {
        return new AutonDownHC();
    }

    public class AutonUpHB implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setTarget(850);
            return false;
        }
    }

    public Action autonUpHB() {
        return new AutonUpHB();
    }





    public double toDegrees(int pos) {
        return (pos * (3.0 / 23.0)) - 42;
    }

    public int fromDegrees(double deg) {
        return (int) (deg / (3.0 / 23.0));
    }


    public void sendTelemetry() {
        myOpMode.telemetry.addData("Arm pos Left/Right", "%4d / %4d",
                shoulder_left.getCurrentPosition(),
                shoulder_right.getCurrentPosition());
        myOpMode.telemetry.addData("Shoulder pidf:", pidf);
        myOpMode.telemetry.addData("Shoulder pos:", armPos);
        myOpMode.telemetry.addData("Shoulder target:", target);
    }


    public double normalize_power(double power) {
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }
        if ( power < MIN_POWER ) {
            power = MIN_POWER;
        }
        return power;
    }


    public void setTarget(int tar) {
        target = tar;
    }

    public int getCurrentPosition() {
        return shoulder_left.getCurrentPosition();
    }

    public void autoListen() {
        int armPos = shoulder_left.getCurrentPosition();
        pidf = controller.calculate(armPos, target);

        shoulder_right.setPower(normalize_power(pidf));
        shoulder_left.setPower(normalize_power(pidf));
    }

    public void listen() {

//        pidf = -myOpMode.gamepad2.right_stick_y;
//
//        shoulder_right.setPower(pidf);
//        shoulder_left.setPower(pidf);
//
//        // move arm according to the left stick y
//
        armPos = shoulder_left.getCurrentPosition();
        boolean controlled = false;

        double right_stick = -myOpMode.gamepad2.right_stick_y;
        boolean override_deadzone = myOpMode.gamepad2.dpad_up;

        if (override_deadzone) {
            target += (int) (right_stick * 50);
            if (target > 950) {
                target = 950;
            }

        } else if (Math.abs(right_stick) > deadzone
        ) {

            if (right_stick > 0) {
                target += (int) (right_stick * 50);
            } else {
                target += (int) (right_stick * 20);
            }

            if (target > 850) {
                target = 850;
            }
            if (target < 0) {
                target = 0;
            }

        }
        armPos = shoulder_left.getCurrentPosition();
        pidf = controller.calculate(armPos, target);

        if (armPos > 850) {
            pidf = pidf - f;
        }


        shoulder_right.setPower(normalize_power(pidf));
        shoulder_left.setPower(normalize_power(pidf));


        if (myOpMode.gamepad2.start) {
            // Reset the target to zero
            target = 0;
            shoulder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            shoulder_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulder_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}