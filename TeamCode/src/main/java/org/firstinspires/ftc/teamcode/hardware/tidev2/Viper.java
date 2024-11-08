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

public class Viper {





    // Define class members

    private DcMotorEx viper;

    private OpMode myOpMode;   // gain access to methods in the calling OpMode.

    private double deg = 0.0;

    private PIDFController controller;

    public static double p = 0.001, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public Viper(OpMode opmode) {
        myOpMode = opmode;
    }


    public static double pidf = 0;





    public void init() {
        controller = new PIDFController(p, i, d, f);
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        viper = myOpMode.hardwareMap.get(DcMotorEx.class, "viper_slide");

        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        viper.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

    }




    public void setTarget(int tar) {
        target = tar;
    }

    public void autoListen() {
        int armPos = viper.getCurrentPosition();
        pidf = controller.calculate(armPos, target);

        viper.setPower(pidf);
        viper.setPower(pidf);
    }



    public void listen() {

        // move viper according to the left stick y


        if (target >=  0 && target <= 5500) {
            target += (int) (-myOpMode.gamepad2.left_stick_y) * 100;
        } else if (target < 0) {
            target = 0;
        } else {
            target = 5500;
        }

        int vipPos = viper.getCurrentPosition();
        pidf = controller.calculate(vipPos, target);

        viper.setPower(pidf);

        myOpMode.telemetry.addData("Viper Position:", vipPos);
        myOpMode.telemetry.addData("Power:", pidf);
        myOpMode.telemetry.addData("Target Position:", target);


        if (myOpMode.gamepad2.dpad_right) {
            viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
}