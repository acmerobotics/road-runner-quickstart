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

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShoulderV0 {

    static final double ramp_y_offset = 0.2;
    static final double ramp_rate = 0.001;
    static final double ramp_a = 0.1;

    static final double deadzone = 0.3;


    // Define class members

    private PIDFController controller;

    public static final double p = 0.003, i = 0.003, d = 0.0001;
    public static final double f = 0.00003;

    public static int target = 100;

    double pidf;


    private OpMode myOpMode;   // gain access to methods in the calling OpMode.


    private DcMotorEx shoulder_right;
    private DcMotorEx shoulder_left;

    public ShoulderV0(OpMode opmode) {
        myOpMode = opmode;
    }


    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        controller = new PIDFController(p, i, d, f);
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
    }




    public void sendTelemetry() {
        myOpMode.telemetry.addData("Arm pos Left/Right", "%4d / %4d",
                shoulder_left.getCurrentPosition(),
                shoulder_right.getCurrentPosition());
    }




    public void listen() {
        int armPos = shoulder_left.getCurrentPosition();
        boolean controlled = false;



        if (Math.abs(myOpMode.gamepad2.right_stick_y) > deadzone
                && armPos <= 850 && armPos >= -100
        )  {
            pidf = -myOpMode.gamepad2.right_stick_y;

            controlled = true;

            if (pidf < 0) {
                // ignores control altogether and come down at a reasonable speed
                pidf = (-ramp_a * Math.exp(ramp_rate * armPos) + ramp_y_offset) + pidf * 0.01;
            }

        } else {
            armPos = shoulder_left.getCurrentPosition();
            pidf = controller.calculate(armPos, target);
        }


        if (controlled) {
            int correction = 100;
            if (pidf < 0) {
                correction = 0;
            }
            target = shoulder_left.getCurrentPosition() + correction;

            if (target > 700) {
                target = 700;
            }
            if (target < 100) {
                target = 100;
            }
        }


        if (myOpMode.gamepad2.y) {
            shoulder_right.setPower(-1);
            shoulder_left.setPower(-1);
        } else {
            shoulder_right.setPower(pidf);
            shoulder_left.setPower(pidf);
        }


        myOpMode.telemetry.addData("pidf", pidf);
        myOpMode.telemetry.addData("pos", armPos);
        myOpMode.telemetry.addData("target", target);




        if (myOpMode.gamepad2.dpad_right) {
            shoulder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }
}