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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shoulder {

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

    private DcMotorEx shoulder_right;
    private DcMotorEx shoulder_left;

    public Shoulder(OpMode opmode) {
        myOpMode = opmode;
    }

    public void setPowerAutoMove(double power) {
        power_auto_move = power;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        shoulder_right = myOpMode.hardwareMap.get(DcMotorEx.class, "left_tower");
        shoulder_left = myOpMode.hardwareMap.get(DcMotorEx.class, "right_tower");

        shoulder_right.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder_left.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    // useless method?
    public int degToPosition(double deg) {
        return (int)(deg / 360 * COUNTS_PER_REVOLUTION * GEAR_RATIO);
    }

    public double positionToDeg(int pos) {
        return (double) shoulder_left.getCurrentPosition() * 360  / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
    }

    public void moveToDegree(double deg) {
        double targetPos = degToPosition(deg);
//        boolean isGoingUp = targetPos > arm_right.getCurrentPosition();
        boolean isGoingUp = deg > 120;


        shoulder_right.setTargetPosition(((int)targetPos));
        shoulder_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        shoulder_left.setTargetPosition(((int)targetPos));
        shoulder_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        shoulder_right.setPower(power_auto_move);
        shoulder_left.setPower(power_auto_move);

        // keep looping while we are still active, and BOTH motors are running.
        while (shoulder_right.isBusy() && shoulder_left.isBusy()) {

            // decide if we reach a threshold to slow down
            if ((isGoingUp
                    && (shoulder_right.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG_HI)
                    || shoulder_left.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG_HI))
            ) || ( !isGoingUp
                    && (shoulder_right.getCurrentPosition() < degToPosition(THRESHOLD_TO_SLOW_IN_DEG_LO)
                    || shoulder_left.getCurrentPosition() < degToPosition(THRESHOLD_TO_SLOW_IN_DEG_LO))
            )) {

                shoulder_right.setPower(0.1);
                shoulder_left.setPower(0.1);
            } else {
                shoulder_right.setPower(power_auto_move);
                shoulder_left.setPower(power_auto_move);
            }

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        shoulder_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulder_right.setPower(0);
        shoulder_left.setPower(0);

        this.deg = deg;

    }

    public void sendTelemetry() {
        myOpMode.telemetry.addData("Arm pos Left/Right", "%4d / %4d",
                shoulder_left.getCurrentPosition(),
                shoulder_right.getCurrentPosition());
    }

    public void moveArmByPower(double power) {
        shoulder_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: allows pushing the arm down for now
        if (true || (power < 0) && (deg > 0)
                || (power > 0) && (deg < 210)) {
            shoulder_right.setPower(power);
            shoulder_left.setPower(power);
        }

        deg = positionToDeg(shoulder_left.getCurrentPosition());
        double contPower = myOpMode.gamepad2.right_stick_y;

        myOpMode.telemetry.addData("Shoulder deg to pos: ", "%.2f", deg);
        myOpMode.telemetry.addData("Shoulder Power: ", "%.2f", contPower);
        sendTelemetry();
    }

    public void listen() {

        // move arm according to the left stick y

//        moveToDegree(30);
        double normPower = myOpMode.gamepad2.right_stick_y;
        double upPower = myOpMode.gamepad2.right_stick_y * 0.8;
        double downPower = -0.1;

        boolean canMove = (shoulder_left.getCurrentPosition() < 480 && shoulder_right.getCurrentPosition() < 480);
        if (canMove && normPower < -0.1) {
            moveArmByPower(upPower);
        } else if (canMove && normPower > 0.1) {
            moveArmByPower(downPower);
        } else {
            moveArmByPower(0);
        }

        if (myOpMode.gamepad2.dpad_right) {
            shoulder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

    }
}