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

package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Control;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.ExtendoV2;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.SlidesV2;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class TestSpeciMacro extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private Telemetry tele = dash.getTelemetry();

    private enum LiftState {LIFTSTART, LIFTDEPOSIT, LIFTWALL, LIFTTOPBAR, LIFTBOTTOMBAR}
    private LiftState liftState = LiftState.LIFTSTART;
    private String clawTelem = "Start";

    private enum ExtendoState {EXTENDOSTART, EXTENDOEXTEND, EXTENDORETRACT}
    private ExtendoState extendoState = ExtendoState.EXTENDOSTART;
    private String extendoTelem = "Start";
    private boolean hasColor = false;

    private boolean intakeUp = false;
    private boolean turning = false;
    private enum HangState {HANGSTART, HANGUP}
    private HangState hangState = HangState.HANGSTART;


    @Override
    public void runOpMode() {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        ExtendoV2 extendo = new ExtendoV2(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);
        SlidesV2 slides = new SlidesV2(hardwareMap, true);
        Claw claw = new Claw(hardwareMap);
        Control slidescontrol = new Control();


        runningActions.add(new SequentialAction(
                intake.flop(),
                claw.flop(),
                claw.open(),
                extendo.retract(),
                slides.retract()
        ));



        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);




            switch (liftState) {
                case LIFTSTART:
                    clawTelem = "Start";
                    if (currentGamepad1.a && !previousGamepad1.a) {
                        runningActions.add(new SequentialAction(
                                slides.retract(),
                                claw.wallClose()
                        ));
                        liftState = LiftState.LIFTWALL;
                    }
                    break;
                case LIFTWALL:
                    clawTelem = "Wall";
                    if (currentGamepad1.b && !previousGamepad1.b) {
                        runningActions.add(new SequentialAction(
                                claw.close(),
                                //new SleepAction(0.3),
                                slides.slideTopBar()
                        ));
                        liftState = LiftState.LIFTTOPBAR;
                    }
                    break;
                case LIFTTOPBAR:
                    clawTelem = "TopBar";
                    if (currentGamepad1.y && !previousGamepad1.y) {
                        runningActions.add(new SequentialAction(
                                slides.slideTopBarClip(),
                                claw.open(),
                                slides.retract()
                        ));
                        liftState = LiftState.LIFTSTART;
                    }
                    break;
                default:
                    liftState = LiftState.LIFTSTART;
                    break;
            }
//
//            if (currentGamepad1.a && !previousGamepad1.a) {
//                    runningActions.add(new SequentialAction(
//                            claw.wallClose()
//                    ));
//                }
//
//            if (currentGamepad1.b && !previousGamepad1.b) {
//                runningActions.add(new SequentialAction(
//                        claw.close(),
//                        new SleepAction(0.3),
//                        slides.slideTopBar(),
//                        slidescontrol.done()
//                ));
//            }
//
//
//
//            if (currentGamepad1.y && !previousGamepad1.y) {
//                runningActions.add(new SequentialAction(
//                        slides.slideTopBarClip(),
//                        claw.open(),
//                        slides.retract(),
//                        slidescontrol.done()
//                ));
//            }


            if (currentGamepad2.dpad_up) {
                slides.changeTarget(-20);
            } else if (currentGamepad2.dpad_down) {
                slides.changeTarget(20);
            }

            slides.updateMotors();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);





            telemetry.addData("claw state", clawTelem);
            telemetry.update();
            tele.addData("claw state", clawTelem);
            tele.update();

        }
    }
}
