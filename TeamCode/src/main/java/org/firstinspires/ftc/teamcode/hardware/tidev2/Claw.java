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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {


    // Define class members
    double torqueClose = 0.25;
    double torqueOpen = 1;

    double speedClose = 0;
    double speedOpen = 0.67;

    ElapsedTime toggle_time = new ElapsedTime();

    private OpMode myOpMode;   // gain access to methods in the calling OpMode.
    boolean pos = false;
    Servo torque;
    Servo speed;
    public Claw(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        torque = myOpMode.hardwareMap.get(Servo.class, "torque");
        speed = myOpMode.hardwareMap.get(Servo.class, "speed");
    }

    public void speedOpen(boolean open) {
        if (open) {
            speed.setPosition(speedOpen);
        } else {
            speed.setPosition(speedClose);
        }
    }

    public void torqueState(boolean open) {
        if (open) {
            torque.setPosition(torqueOpen);
        } else {
            torque.setPosition(torqueClose);
        }
    }


    public void listen() {

        torque.setPosition(torqueClose);

        if (myOpMode.gamepad2.x && toggle_time.seconds() > 0.5) {
            toggle_time.reset();
            pos = !pos;
        }

        if(pos) {
//            torque.setPosition(torqueClose);
            speed.setPosition(speedClose);
        } else {
//            torque.setPosition(torqueOpen);
            speed.setPosition(speedOpen);
        }
    }

    public void sendTelemetry() {
        myOpMode.telemetry.addData("Claw Position: ",pos);
    }
}