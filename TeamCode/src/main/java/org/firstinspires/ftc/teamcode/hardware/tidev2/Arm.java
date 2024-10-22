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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

public class Arm {

    static final double ARM_UP_POS = -1;
    static final double ARM_DOWN_POS = 1;
    static final int SLEEP_TIME = 500;

    static final int COUNTS_PER_REVOLUTION = 288;
    static final double GEAR_RATIO = 40 / 10;


    private double power_auto_move = 0.6;

    static final double THRESHOLD_TO_SLOW_IN_DEG_HI = 70;
    static final double THRESHOLD_TO_SLOW_IN_DEG_LO = 40;


    static final double  POWER_UP_MUL = 0.8;
    static final double  POWER_DOWN_MUL = 0.8;
    // Define class members


    private OpMode myOpMode;   // gain access to methods in the calling OpMode.

    private double deg = 0.0;

    private DcMotor arm_right = null;
    private DcMotor arm_left = null;

    public Arm (OpMode opmode) {
        myOpMode = opmode;
    }

    public void setPowerAutoMove(double power) {
        power_auto_move = power;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        arm_right = myOpMode.hardwareMap.get(DcMotor.class, "arm_right");
        arm_left = myOpMode.hardwareMap.get(DcMotor.class, "arm_left");

        arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveArmUp() {
        deg = 150;
        moveToDegree(deg);
    }

    public void moveArmUpMore() {
        deg = 180;
        moveToDegree(deg);
    }

    public void moveArmDown() {
        deg = 0.0;
        moveToDegree(deg);
    }

    public boolean isArmUp() {
        if (deg >= 100) {
            return true;
        }
        return false;
    }

    public int degToPosition(double deg) {
        return (int)(deg / 360 * COUNTS_PER_REVOLUTION * GEAR_RATIO);
    }

    public double positionToDeg(int pos) {
        return (double) arm_right.getCurrentPosition() * 360  / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
    }

    public void moveToDegree(double deg) {
        double targetPos = degToPosition(deg);
//        boolean isGoingUp = targetPos > arm_right.getCurrentPosition();
        boolean isGoingUp = deg > 120;


        arm_right.setTargetPosition(((int)targetPos));
        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        arm_left.setTargetPosition(((int)targetPos));
        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        arm_right.setPower(power_auto_move);
        arm_left.setPower(power_auto_move);

        // keep looping while we are still active, and BOTH motors are running.
        while (arm_right.isBusy() && arm_left.isBusy()) {

            // decide if we reach a threshold to slow down
            if ((isGoingUp
                    && (arm_right.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG_HI)
                    || arm_left.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG_HI))
            ) || ( !isGoingUp
                    && (arm_right.getCurrentPosition() < degToPosition(THRESHOLD_TO_SLOW_IN_DEG_LO)
                    || arm_left.getCurrentPosition() < degToPosition(THRESHOLD_TO_SLOW_IN_DEG_LO))
            )) {

                arm_right.setPower(0.1);
                arm_left.setPower(0.1);
            } else {
                arm_right.setPower(power_auto_move);
                arm_left.setPower(power_auto_move);
            }

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        arm_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm_right.setPower(0);
        arm_left.setPower(0);

        this.deg = deg;

    }

    public void sendTelemetry() {
        myOpMode.telemetry.addData("Arm pos Left/Right", "%4d / %4d",
                arm_left.getCurrentPosition(),
                arm_right.getCurrentPosition());
    }

    public void moveArmByPower(double power) {
        arm_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: allows pushing the arm down for now
        if (true || (power < 0) && (deg > 0)
                || (power > 0) && (deg < 210)) {
            arm_right.setPower(power);
            arm_left.setPower(power);
        }

        deg = positionToDeg(arm_left.getCurrentPosition());

        myOpMode.telemetry.addData("Arm deg: ", "%.2f", deg);
        sendTelemetry();
    }

    public void listen() {

        // move arm according to the left stick y


        double power = -myOpMode.gamepad2.right_stick_y;
        if (Math.abs(power) > 0.1) {
            moveArmByPower(power);
        } else {
            arm_right.setPower(0.0);
            arm_left.setPower(0.0);
        }

        if (myOpMode.gamepad2.dpad_right) {
            arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }
}