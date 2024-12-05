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

package org.firstinspires.ftc.teamcode.hardware.tidev2.automation;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;

public class SubOperatorFSM {

    private final int POS_SHOULDER_SUB = 250;
    private final int POS_SHOULDER_LOWER = 375;
    private final int POS_SHOULDER_HANG = 900;
    private final int THRESH_SHOULDER = 150;
    private final int THRESH_VIPER = 50;
    private final int THRESH_ELBOW = 50;

    private final int POS_VIPER_SUB = 1000;
    private final int POS_VIPER_HANG = 4269;

    private final int POS_ELBOW_EXTEND_HORIZ_SUB = 450;
    private final int POS_ELBOW_EXTEND_ADJUST_SUB = 450;
    private final int POS_ELBOW_EXTEND_MAX_SUB = 650;
    private final int POS_ELBOW_REST = 200;



    private Viper viper;
    private Elbow elbow;
    private Shoulder shoulder;
    private Claw claw;

    private Gamepad gamepad;

    public enum SubState {
        ZERO_SUBSTATE,
        SHOULDER_RAISE_SUBSTATE, HORIZ_ELBOW_SUBSTATE, VIPER_EXTEND_SUBSTATE, MAX_ELBOW_SUBSTATE,
        RETRACT_VIPER_SUBSTATE, RETRACT_ELBOW_SUBSTATE_S1, RETRACT_ELBOW_SUBSTATE_S2, CLEARANCE_ELBOW_SUBSTATE,
        BEGIN_HANG_SUBSTATE, PROCEED_HANG_SUBSTATE
    }
    private ElapsedTime subStateTimer = new ElapsedTime();
    private SubState subState;

    public SubOperatorFSM(Gamepad gamepad, Shoulder shoulder, Viper viper, Elbow elbow, Claw claw, Intake intake) {
        this.gamepad = gamepad;
        this.shoulder = shoulder;
        this.viper = viper;
        this.elbow = elbow;
        this.claw = claw;

        subState = SubState.ZERO_SUBSTATE;
        subStateTimer = new ElapsedTime();
    }

    public void listen() {
        int pos_elbow;
        int pos_shoulder;
        int pos_viper;

        // listen to the command to reset
        if (subStateTimer.seconds() > 1) {
            switch (subState) {
                case ZERO_SUBSTATE:
                    break;

                case SHOULDER_RAISE_SUBSTATE:
                case VIPER_EXTEND_SUBSTATE:
                    if (gamepad.b) {
                        elbow.setElbow(0);
                        subStateTimer.reset();
                        subState = SubState.RETRACT_ELBOW_SUBSTATE_S2;
                    }
                    break;

                case HORIZ_ELBOW_SUBSTATE:
                case MAX_ELBOW_SUBSTATE:
                case CLEARANCE_ELBOW_SUBSTATE:
                    if (gamepad.b) {
                        elbow.setElbow(POS_ELBOW_REST);
                        subStateTimer.reset();
                        subState = SubState.RETRACT_ELBOW_SUBSTATE_S1;
                    }
                    break;
                case BEGIN_HANG_SUBSTATE:
                    if (gamepad.b) {
                        elbow.setElbow(0);
                        claw.customPivotPos(claw.PIVOT_NORMAL);
                        viper.setTarget(0);
                        subStateTimer.reset();
                        subState = SubState.RETRACT_VIPER_SUBSTATE;
                    }
                    break;
                case PROCEED_HANG_SUBSTATE:
                    if (gamepad.b) {
                        elbow.setElbow(0);
                        claw.customPivotPos(claw.PIVOT_NORMAL);
                        subStateTimer.reset();
                        viper.setTarget(0);
                        subState = SubState.RETRACT_VIPER_SUBSTATE;
                    }

                        break;


            }
        }

        // listen to commands
        switch (subState) {
            case ZERO_SUBSTATE:
                if (gamepad.dpad_down) {
                    // go to the shoulder sub pos
                    claw.customPivotPos(claw.PIVOT_HIGH);
                    shoulder.setTarget(POS_SHOULDER_SUB);
                    subState = SubState.SHOULDER_RAISE_SUBSTATE;
                    subStateTimer.reset();
                }
                if (gamepad.y) {
                    subState = SubState.BEGIN_HANG_SUBSTATE;
                    shoulder.setTarget(POS_SHOULDER_HANG);

                }

                break;


            case SHOULDER_RAISE_SUBSTATE:
                pos_shoulder = shoulder.getCurrentPosition();

                // stay until the shoulder is within threshold
                if (subStateTimer.seconds() > 0.3) {
                    //go to horiz pos
                    elbow.setElbow(POS_ELBOW_EXTEND_HORIZ_SUB);
                    subStateTimer.reset();
                    subState = SubState.HORIZ_ELBOW_SUBSTATE;

                }
                break;

            case HORIZ_ELBOW_SUBSTATE:
                pos_elbow = elbow.getPosition();


                // stay until elbow is within thresh
                if (subStateTimer.seconds() > 0.3) {

                    //extend viper
                    viper.setTarget(POS_VIPER_SUB);
                    subStateTimer.reset();
                    subState = SubState.VIPER_EXTEND_SUBSTATE;
                }
                break;

            case VIPER_EXTEND_SUBSTATE:
                pos_viper = viper.getPosition();


                // stay until viper is within thresh
                if (subStateTimer.seconds() > 0.3 || (pos_viper >= POS_VIPER_SUB - THRESH_VIPER) && (pos_viper <= POS_VIPER_SUB + THRESH_VIPER)) {
                    //go to max pos
                    if (gamepad.a) {
                        elbow.setElbow(POS_ELBOW_EXTEND_MAX_SUB);
                        shoulder.setTarget(POS_SHOULDER_LOWER);
                        subStateTimer.reset();
                        subState = SubState.MAX_ELBOW_SUBSTATE;
                    }
                }

                break;

            case MAX_ELBOW_SUBSTATE:
                pos_elbow = elbow.getPosition();

                // stay until elbow is within thresh
                if (subStateTimer.seconds() > 0.3) {

                    if (gamepad.a) {
                        //go to max pos
                        elbow.setElbow(POS_ELBOW_EXTEND_ADJUST_SUB);
                        shoulder.setTarget(POS_SHOULDER_SUB);
                        subStateTimer.reset();
                        subState = SubState.CLEARANCE_ELBOW_SUBSTATE;

                    }

                }

                break;

            case RETRACT_VIPER_SUBSTATE:
                pos_viper = viper.getPosition();

                // stay until viper is within thresh
                if (subStateTimer.seconds() > 0.3 || pos_viper <= +THRESH_VIPER) {
                    subState = SubState.ZERO_SUBSTATE;
                }
                break;

            case RETRACT_ELBOW_SUBSTATE_S1:
                if (subStateTimer.seconds() > 0.3) {
                    elbow.setElbow(0);
                    subStateTimer.reset();
                    subState = SubState.RETRACT_ELBOW_SUBSTATE_S2;
                }
                break;

            case RETRACT_ELBOW_SUBSTATE_S2:
                pos_elbow = elbow.getPosition();

                if (subStateTimer.seconds() > 0.3 || pos_elbow <= THRESH_ELBOW) {
                    viper.setTarget(0);
                    subStateTimer.reset();
                    subState = SubState.RETRACT_VIPER_SUBSTATE;
                }
                break;

            case CLEARANCE_ELBOW_SUBSTATE:
                pos_elbow = elbow.getPosition();


                // stay until elbow is within thresh
                if (subStateTimer.seconds() > 0.3) {

                    if (gamepad.a) {
                        shoulder.setTarget(POS_SHOULDER_LOWER);
                        elbow.setElbow(POS_ELBOW_EXTEND_MAX_SUB);
                        subStateTimer.reset();
                        subState = SubState.MAX_ELBOW_SUBSTATE;

                    }

                }

                break;


                //Jonathan Hanging Stance changes
            case BEGIN_HANG_SUBSTATE:




                if (subStateTimer.seconds() > 3.0) {
                    if (gamepad.a) {
                        elbow.setElbow(375);
                        subStateTimer.reset();
                        subState = SubState.PROCEED_HANG_SUBSTATE;
                    }

                }
                break;

            case PROCEED_HANG_SUBSTATE:
                if (subStateTimer.seconds() > 1.0) {
                    if (gamepad.y) {
                        shoulder.setTarget(900);
                        viper.setTarget(4269);

                    }
                    if (gamepad.x) {
                        viper.setTarget(0);
                    }
                }



                break;

        }
    }
}


