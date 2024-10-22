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

package org.firstinspires.ftc.teamcode.hardware.tidev1_DO_NOT_USE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class Mouth {


    // Define class members



    private OpMode myOpMode;   // gain access to methods in the calling OpMode.

    double left_trigger;
    double right_trigger;
    double power;



    CRServo mouth = null;
    public Mouth(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        mouth = myOpMode.hardwareMap.get(CRServo.class, "mouth");


    }



    public void listen() {
        left_trigger = myOpMode.gamepad2.left_trigger;
        right_trigger = myOpMode.gamepad2.right_trigger;

        power = -left_trigger + right_trigger;

        mouth.setPower(power);


        myOpMode.telemetry.addData("Left Trigger", "%.2f", left_trigger);
        myOpMode.telemetry.addData("Right Trigger", "%.2f", right_trigger);
        myOpMode.telemetry.addData("Power", "%.2f", power);

    }


    public void sendTelemetry() {
        myOpMode.telemetry.addData("Left Trigger", "%.2f", left_trigger);
        myOpMode.telemetry.addData("Right Trigger", "%.2f", right_trigger);
        myOpMode.telemetry.addData("Power", "%.2f", power);
    }
}