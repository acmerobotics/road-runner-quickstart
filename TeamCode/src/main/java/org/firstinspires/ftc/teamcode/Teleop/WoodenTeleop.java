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

package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class WoodenTeleop extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {

        Extendo extendo = new Extendo(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean intakeFlipped = false;
        boolean clawFlipped = false;


        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double y = currentGamepad1.left_stick_y;
            double x = -currentGamepad1.left_stick_x;
            double rx = -currentGamepad1.right_stick_x;

            double lefty = currentGamepad2.left_stick_y;
            double righty = currentGamepad2.right_stick_y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            extendo.extendoMotor.setPower(lefty/1.5);

            slides.slidesRightMotor.setPower(righty/3);
            slides.slidesLeftMotor.setPower(righty/3);

            if (currentGamepad2.x) {
                runningActions.add(intake.intake());
            } else if (currentGamepad2.b) {
                runningActions.add(intake.extake());
            } else {
                runningActions.add(intake.off());
            }

            if (currentGamepad2.a && !previousGamepad2.a) {
                if (!intakeFlipped) {
                    runningActions.add(intake.flip());
                    intakeFlipped = true;
                } else {
                    runningActions.add(intake.flop());
                    intakeFlipped = false;
                }
            }

            if (currentGamepad2.y && !previousGamepad2.y) {
                if (!clawFlipped) {
                    runningActions.add(claw.flip());
                    clawFlipped = true;
                } else {
                    runningActions.add(claw.flop());
                    clawFlipped = false;
                }
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

        }
    }
}
