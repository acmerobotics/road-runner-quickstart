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
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shoulder {

    public enum BucketState{
        ZERO_BUCKETSTATE,
        MIDDLE_BUCKETSTATE,
        HIGH_BUCKETSTATE
    };

    private BucketState bucketState;

    private ElapsedTime bucketStateTimer = new ElapsedTime();

    static final double deadzone = 0.3;


    private PIDFController controller;

    public static final double p = 0.003, i = 0.003, d = 0.0001;
    public static final double f = 0.00003;

    public static int target = 100;

    double pidf;


    private OpMode myOpMode;   // gain access to methods in the calling OpMode.


    private DcMotorEx shoulder_right;
    private DcMotorEx shoulder_left;

    public Shoulder(OpMode opmode) {
        myOpMode = opmode;
    }


    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        bucketState = BucketState.ZERO_BUCKETSTATE;
        bucketStateTimer = new ElapsedTime();

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

        target = 100;
    }








    public void sendTelemetry() {
        myOpMode.telemetry.addData("Arm pos Left/Right", "%4d / %4d",
                shoulder_left.getCurrentPosition(),
                shoulder_right.getCurrentPosition());
    }



    public void setTarget(int tar) {
        target = tar;
    }

    public void autoListen() {
        int armPos = shoulder_left.getCurrentPosition();
        pidf = controller.calculate(armPos, target);

        shoulder_right.setPower(pidf);
        shoulder_left.setPower(pidf);
    }

    public void listen() {
//        pidf = -myOpMode.gamepad2.right_stick_y;
//
//        shoulder_right.setPower(pidf);
//        shoulder_left.setPower(pidf);
//
//        // move arm according to the left stick y
//
        int armPos = shoulder_left.getCurrentPosition();
        boolean controlled = false;

        double right_stick = -myOpMode.gamepad2.right_stick_y;
        boolean override_deadzone = myOpMode.gamepad2.dpad_up;
        boolean to_bucket = myOpMode.gamepad2.dpad_left;

        if(to_bucket) {
            if (bucketStateTimer.seconds() >= 1.0) {
                switch (bucketState) {
                    case ZERO_BUCKETSTATE:
                    case HIGH_BUCKETSTATE:
                        target = 620;
                        bucketStateTimer.reset();
                        bucketState = BucketState.MIDDLE_BUCKETSTATE;
                        break;

                    case MIDDLE_BUCKETSTATE:
                        target = 790;
                        bucketStateTimer.reset();
                        bucketState = BucketState.HIGH_BUCKETSTATE;
                        break;
                }
            }
        }

        if (override_deadzone) {
            target += right_stick * 50;

        } else if (Math.abs(right_stick) > deadzone
                && armPos <= 950 && armPos >= -100
        ) {
            bucketState = BucketState.ZERO_BUCKETSTATE;

            if (right_stick > 0) {
                target += (int) right_stick * 50;
            } else {
                target += (int) right_stick * 10;
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


        shoulder_right.setPower(pidf);
        shoulder_left.setPower(pidf);



        myOpMode.telemetry.addData("pidf", pidf);
        myOpMode.telemetry.addData("pos", armPos);
        myOpMode.telemetry.addData("target", target);



        if (myOpMode.gamepad2.dpad_right) {
            shoulder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            shoulder_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shoulder_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
}