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
import androidx.appcompat.widget.ActionBarOverlayLayout;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Elbow {

    public final double MAX_POWER = 1.0;
    public final double MIN_POWER = -1.0;

    public final int MAX_ELBOW = 650;
    public final int MAX_STRETCH = 750;

    private PIDFController controller;

    public static double p = 0.005, i = 0.0013, d = 0.0002;
    public static double f = 0;

    public static int target = 0;
    // Define class members

    private DcMotorEx elbow;

    private OpMode myOpMode;   // gain access to methods in the calling OpMode.

    int elbPos;

    double pidf;

//    PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);

    public Elbow(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(5, 10);

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        elbow = myOpMode.hardwareMap.get(DcMotorEx.class, "elbow");

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setDirection(DcMotor.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        elbow.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        target = 0;

    }

    public int getPosition() {
        return elbow.getCurrentPosition();
    }

    public void setElbow(int tar) {
        target = tar;
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

    public void autoListen() {

        elbPos = elbow.getCurrentPosition();
        pidf = controller.calculate(elbPos, target);

        elbow.setPower(normalize_power(pidf));

    }

    public void sendTelemetry() {
        elbPos = elbow.getCurrentPosition();

        myOpMode.telemetry.addData("Elbow Position:", elbPos);
        myOpMode.telemetry.addData("Power:", pidf);
        myOpMode.telemetry.addData("Target Position:", target);
    }

    public void listen_simple() {
        pidf = -myOpMode.gamepad2.right_trigger +myOpMode.gamepad2.left_trigger;
        if (pidf != 0.0) {
            elbow.setPower(normalize_power(pidf));
            target = elbow.getCurrentPosition();
        }
    }

    public void listen() {
        listen_complex();
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


    public class AutonHB implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setElbow(600);
            return false;
        }
    }
    public Action autonHB() {
        return new AutonHB();
    }

    public class AutonPick implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setElbow(200);
            setElbow(400);
            setElbow(550);
            return false;
        }
    }

    public Action autonPick() {
        return new AutonPick();
    }

    public class AutonZero implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setElbow(0);
            return false;
        }
    }

    public Action autonZero() {
        return new AutonZero();}




    public void listen_complex() {

        // accommodate the override
        if (myOpMode.gamepad2.dpad_up) {
            listen_simple();

        } else {

            target += (int) (-(myOpMode.gamepad2.right_trigger - myOpMode.gamepad2.left_trigger) * 10);

            if (target < 0) {
                target = 0;
            }
            if (target > MAX_STRETCH) {
                target = MAX_STRETCH;
            }

            elbPos = elbow.getCurrentPosition();
            pidf = controller.calculate(elbPos, target);

            elbow.setPower(normalize_power(pidf));

        }

        if (myOpMode.gamepad2.start) {
            // Reset the target to zero
            target = 0;

            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
}