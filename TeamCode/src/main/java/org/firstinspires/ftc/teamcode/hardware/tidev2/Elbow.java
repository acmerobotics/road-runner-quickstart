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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Elbow {



    private PIDFController controller;

    public static double p = 0.001, i = 0.0001, d = 0.0001;
    public static double f = 0;

    public static int target = 0;
    // Define class members

    private DcMotorEx elbow;

    private OpMode myOpMode;   // gain access to methods in the calling OpMode.


//    PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

    public Elbow(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        controller = new PIDFController(p, i, d, f);
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        elbow = myOpMode.hardwareMap.get(DcMotorEx.class, "elbow");

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elbow.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setElbow(int tar) {
        target = tar;
    }

    public void autoListen() {

        int elbPos = elbow.getCurrentPosition();
        double pidf = controller.calculate(elbPos, target);

        elbow.setPower(pidf);

    }

    public void listen() {

        if (target <=  0 && target >= -600) {
            target += (int) (myOpMode.gamepad2.right_trigger - myOpMode.gamepad2.left_trigger) * 50;
        } else if (target > 0) {
            target = 0;
        } else {
            target = -600;
        }

        int elbPos = elbow.getCurrentPosition();
        double pidf = controller.calculate(elbPos, target);

        elbow.setPower(pidf);

        myOpMode.telemetry.addData("Elbow Position:", elbPos);
        myOpMode.telemetry.addData("Power:", pidf);
        myOpMode.telemetry.addData("Target Position:", target);


        if (myOpMode.gamepad2.dpad_right) {
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
}