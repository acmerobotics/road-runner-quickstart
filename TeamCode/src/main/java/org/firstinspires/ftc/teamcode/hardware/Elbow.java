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

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Elbow {

    static final double ELBOW_UP_POS = -1; //Placeholder
    static final double ARM_DOWN_POS = 1; //Placeholder
    static final int SLEEP_TIME = 500;

    static final double COUNTS_PER_REVOLUTION = 5700.4; //Placeholder
    static final double GEAR_RATIO = 50.9 / 1; //Placeholder


    private double power_auto_move = 0.6;

    static final double THRESHOLD_TO_SLOW_IN_DEG_HI = 70;
    static final double THRESHOLD_TO_SLOW_IN_DEG_LO = 40;


    static final double  POWER_UP_MUL = 0.8;
    static final double  POWER_DOWN_MUL = 0.8;
    // Define class members

    private DcMotor elbow;

    private OpMode myOpMode;   // gain access to methods in the calling OpMode.

    private double deg = 0.0;

    public Elbow(OpMode opmode) {
        myOpMode = opmode;
    }

    public void setPowerAutoMove(double power) {
        power_auto_move = power;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        elbow = myOpMode.hardwareMap.get(DcMotor.class, "elbow");

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        return (double) elbow.getCurrentPosition() * 360  / (COUNTS_PER_REVOLUTION * GEAR_RATIO);
    }

    public void moveToDegree(double deg) {
        double targetPos = degToPosition(deg);
//        boolean isGoingUp = targetPos > arm_right.getCurrentPosition();
        boolean isGoingUp = deg > 120;


        elbow.setTargetPosition(((int)targetPos));
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        elbow.setPower(power_auto_move);

        // keep looping while we are still active, and BOTH motors are running.
        while (elbow.isBusy()) {

            // decide if we reach a threshold to slow down
            if ((isGoingUp
                    && (elbow.getCurrentPosition() > degToPosition(THRESHOLD_TO_SLOW_IN_DEG_HI)
                    )
            ) || ( !isGoingUp
                    && (elbow.getCurrentPosition() < degToPosition(THRESHOLD_TO_SLOW_IN_DEG_LO)
                    )
            )) {

                elbow.setPower(0.1);
            } else {
                elbow.setPower(power_auto_move);
                            }

            // Display drive status for the driver.
            sendTelemetry();
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow.setPower(0);
        this.deg = deg;

    }

    public void sendTelemetry() {
        myOpMode.telemetry.addData("Viper Degree", "%4d",
                elbow.getCurrentPosition());
    }

    public void moveArmByPower(double power) {
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if (true || (power < 0) && (deg > 0)
                || (power > 0) && (deg < 210)) {
            elbow.setPower(power);
        }

        deg = positionToDeg(elbow.getCurrentPosition());

        myOpMode.telemetry.addData("Viper deg: ", "%.2f", deg);
        sendTelemetry();
    }

    public void listen() {

        // move arm according to the left stick y


        double power = -myOpMode.gamepad2.right_stick_y / 10;
        if (Math.abs(power) > 0.1) {
            moveArmByPower(power);
        } else {
            elbow.setPower(0.0);
            elbow.setPower(0.0);
        }

        if (myOpMode.gamepad2.dpad_right) {
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

    }
}