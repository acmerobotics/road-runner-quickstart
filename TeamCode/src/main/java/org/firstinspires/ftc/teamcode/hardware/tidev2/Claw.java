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

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    public enum PivotPosState {
        NORMAL, HIGH, LOW, CUSTOM
    }

    // Define class members
    public final double PIVOT_LOW = 0.68;
    public final double PIVOT_NORMAL = 0.85;
    public final double PIVOT_HIGH = 1.0;
    double wildcard;

    double clawOpen = 0.3;
    double clawClose = 0.68;

    ElapsedTime toggle_time = new ElapsedTime();
    ElapsedTime pivot_delay = new ElapsedTime();

    private OpMode myOpMode;   // gain access to methods in the calling OpMode.
    boolean pos = false;
    PivotPosState pivotpos = PivotPosState.LOW;
    Servo pivot;
    Servo claw;
    public Claw(OpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // turn torque to pivot in preparation for claw update
        pivot = myOpMode.hardwareMap.get(Servo.class, "pivot");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        pivotpos = PivotPosState.LOW;
    }

    public void speedOpen(boolean open) {
        if (open) {

            pos = true;
        } else {

            pos = false;
        }
    }

    public void setPivot(double pos) {
        pivot.setPosition(pos);
    }

    public void customPivotPos(double pos) {
        pivotpos = PivotPosState.CUSTOM;
        wildcard = pos;
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


    public class AutonNormalPivot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setPivot(0);
            return false;
        }
    }
    public Action autonNormalPivot() {
        return new AutonNormalPivot();
    }

    public class AutonFlushPivot implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPivot(1);
            return false;
        }
    }
    public Action autonFlushPivot() {
        return new AutonFlushPivot();
    }

    public class AutonOpenClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pos = true;
            return false;
        }
    }
    public Action autonOpenClaw() {
        return new AutonOpenClaw();
    }

    public class AutonCloseClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pos = false;
            return false;
        }
    }
    public Action autonCloseClaw() {
        return new AutonCloseClaw();
    }

    public void autoListen() {
        if (pos) {
            claw.setPosition(clawOpen);
        } else {
            claw.setPosition(clawClose);
        }
        switch (pivotpos) {
            case NORMAL:
                pivot.setPosition(PIVOT_NORMAL);
                break;
            case HIGH:
                pivot.setPosition(PIVOT_HIGH);
                break;
            case LOW:
                pivot.setPosition(PIVOT_LOW);
                break;
            case CUSTOM:
                pivot.setPosition(wildcard);
                break;
        }
    }


    public void listen() {


        if (myOpMode.gamepad2.x && toggle_time.seconds() > 0.5) {
            toggle_time.reset();
            pos = !pos;
        }

        if (myOpMode.gamepad2.back && pivot_delay.seconds() > 0.2) {
            pivot_delay.reset();
            switch(pivotpos) {
                case LOW:
                    pivotpos = PivotPosState.HIGH;
                    break;
                case NORMAL:
                    pivotpos = PivotPosState.LOW;
                    break;
                case HIGH:
                    pivotpos = PivotPosState.NORMAL;
                    break;
                case CUSTOM:
                    pivotpos = PivotPosState.LOW;
                    break;

            }

        }

        if (pos) {
            claw.setPosition(clawOpen);
        } else {
            claw.setPosition(clawClose);
        }
        switch (pivotpos) {
            case NORMAL:
                pivot.setPosition(PIVOT_NORMAL);
                break;
            case HIGH:
                pivot.setPosition(PIVOT_HIGH);
                break;
            case LOW:
                pivot.setPosition(PIVOT_LOW);
                break;
            case CUSTOM:
                pivot.setPosition(wildcard);
                break;

        }
    }

    public void sendTelemetry() {
        myOpMode.telemetry.addData("Claw Pivot Position: ", pivotpos);
        myOpMode.telemetry.addData("Claw Position: ", pos);
    }
}