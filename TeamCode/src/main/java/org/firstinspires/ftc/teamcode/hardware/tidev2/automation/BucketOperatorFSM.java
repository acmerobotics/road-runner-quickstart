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

public class BucketOperatorFSM {

    private final int POS_SHOULDER_MIDDLE_BUCKET = 620;
    private final int POS_SHOULDER_HIGH_BUCKET = 850;
    private final int THRESH_SHOULDER = 120;
    private final int THRESH_VIPER = 50;
    private final int THRESH_ELBOW = 15;

    private final int POS_VIPER_MIDDLE_BUCKET = 1000;
    private final int POS_VIPER_HIGH_BUCKET = 3000;

    private final int POS_ELBOW_EXTEND_HIGH_BUCKET = 575;
    private final int POS_ELBOW_EXTEND_LOW_BUCKET = 500;
    private final int POS_ELBOW_REST = 200;

    private final double LAPSE_SEC_ELBOW_S1 = 0.5;


    private Viper viper;
    private Elbow elbow;
    private Claw claw;
    private Intake intake;
    private Shoulder shoulder;

    private Gamepad gamepad;

    public enum BucketState{
        ZERO_BUCKETSTATE,
        MIDDLE_BUCKETSTATE, MIDDLE_EXTEND_VIPER, MIDDLE_EXTEND_ELBOW, MIDDLE_RETRACT_VIPER, MIDDLE_RETRACT_ELBOW_S1, MIDDLE_RETRACT_ELBOW_S2,
        HIGH_BUCKETSTATE, HIGH_EXTEND_VIPER, HIGH_EXTEND_ELBOW, HIGH_RETRACT_VIPER, HIGH_RETRACT_ELBOW_S1, HIGH_RETRACT_ELBOW_S2,

    };
    private ElapsedTime bucketStateTimer = new ElapsedTime();
    private BucketState bucketState;

    public BucketOperatorFSM(Gamepad gamepad, Shoulder shoulder, Viper viper, Elbow elbow, Claw claw, Intake intake) {
        this.gamepad = gamepad;
        this.shoulder = shoulder;
        this.viper = viper;
        this.elbow = elbow;
        this.claw = claw;
        this.intake = intake;

        bucketState = BucketState.ZERO_BUCKETSTATE;
        bucketStateTimer = new ElapsedTime();
    }

    public void listen() {
        int pos_elbow;
        int pos_shoulder;
        int pos_viper;

        // listen to the command to reset
        if (bucketStateTimer.seconds() > 1) {
            switch (bucketState) {
                case ZERO_BUCKETSTATE:
                    break;

                case HIGH_BUCKETSTATE:
                    if (gamepad.b) {
                        elbow.setElbow(0);
                        bucketStateTimer.reset();
                        bucketState = BucketState.HIGH_RETRACT_ELBOW_S2;
                    }
                    break;

                case HIGH_EXTEND_VIPER:
                case HIGH_EXTEND_ELBOW:
                    if (gamepad.b) {
                        elbow.setElbow(POS_ELBOW_REST);
                        bucketStateTimer.reset();
                        bucketState = BucketState.HIGH_RETRACT_ELBOW_S1;
                    }
                    break;

                case MIDDLE_BUCKETSTATE:
                    if (gamepad.b) {
                        elbow.setElbow(0);
                        bucketStateTimer.reset();
                        bucketState = BucketState.MIDDLE_RETRACT_ELBOW_S2;
                    }
                    break;

                case MIDDLE_EXTEND_VIPER:
                case MIDDLE_EXTEND_ELBOW:
                    if (gamepad.b) {
                        elbow.setElbow(POS_ELBOW_REST);
                        bucketStateTimer.reset();
                        bucketState = BucketState.MIDDLE_RETRACT_ELBOW_S1;
                    }
                    break;
            }
        }

        // listen to other commands
        switch (bucketState) {
            case ZERO_BUCKETSTATE:
                if (gamepad.dpad_left) {
                    // go to middle bucket
                    shoulder.setTarget(POS_SHOULDER_MIDDLE_BUCKET);
                    bucketState = BucketState.MIDDLE_BUCKETSTATE;
                    bucketStateTimer.reset();
                }
                break;

            case HIGH_BUCKETSTATE:
                pos_shoulder = shoulder.getCurrentPosition();

                // stay until the shoulder is within threshold
                if (bucketStateTimer.seconds() > 0.3
                        && (pos_shoulder >= POS_SHOULDER_HIGH_BUCKET-THRESH_SHOULDER)
                        && (pos_shoulder <= POS_SHOULDER_HIGH_BUCKET+THRESH_SHOULDER)) {
                    if (gamepad.dpad_left) {
                        // go to the high bucket
                        shoulder.setTarget(POS_SHOULDER_MIDDLE_BUCKET);
                        bucketStateTimer.reset();
                        bucketState = BucketState.MIDDLE_BUCKETSTATE;
                    }
                    if (gamepad.dpad_right) {
                        // extend to drop sample
                        viper.setTarget(POS_VIPER_HIGH_BUCKET);
                        bucketState = BucketState.HIGH_EXTEND_VIPER;
                    }
                }

                if (gamepad.right_stick_y != 0.0) {
                    bucketState = BucketState.ZERO_BUCKETSTATE;
                }
                break;


            case MIDDLE_BUCKETSTATE:
                pos_shoulder = shoulder.getCurrentPosition();

                // stay until the shoulder is within threshold
                if (bucketStateTimer.seconds() > 0.3
                        && (pos_shoulder >= POS_SHOULDER_MIDDLE_BUCKET-THRESH_SHOULDER)
                        && (pos_shoulder <= POS_SHOULDER_MIDDLE_BUCKET+THRESH_SHOULDER)) {
                    if (gamepad.dpad_left) {
                        // go to the high bucket
                        shoulder.setTarget(POS_SHOULDER_HIGH_BUCKET);
                        bucketStateTimer.reset();
                        bucketState = BucketState.HIGH_BUCKETSTATE;
                    }
                    if (gamepad.dpad_right) {
                        // extend to drop sample
                        viper.setTarget(POS_VIPER_MIDDLE_BUCKET);
                        bucketState = BucketState.MIDDLE_EXTEND_VIPER;
                    }
                }
                if (gamepad.right_stick_y != 0.0) {
                    bucketState = BucketState.ZERO_BUCKETSTATE;
                }
                break;

            case MIDDLE_EXTEND_VIPER:
                pos_viper = viper.getPosition();

                // stay until the shoulder is within threshold
                if ((pos_viper >= POS_VIPER_MIDDLE_BUCKET-THRESH_VIPER)
                        && (pos_viper <= POS_VIPER_MIDDLE_BUCKET+THRESH_VIPER)) {

                    // extend the elbow
                    elbow.setElbow(POS_ELBOW_EXTEND_LOW_BUCKET);
                    bucketState = BucketState.MIDDLE_EXTEND_ELBOW;
                }

                break;

            case MIDDLE_EXTEND_ELBOW:
                pos_elbow = elbow.getPosition();

                // stay until the elbow is within threshold
                if ((pos_elbow >= POS_ELBOW_EXTEND_LOW_BUCKET -THRESH_ELBOW)
                        && (pos_elbow <= POS_ELBOW_EXTEND_LOW_BUCKET +THRESH_ELBOW)) {

                    // listen to the command to retract
                    if (gamepad.b) {
                        elbow.setElbow(POS_ELBOW_REST);
                        bucketStateTimer.reset();
                        bucketState = BucketState.MIDDLE_RETRACT_ELBOW_S1;
                    }
                }

                break;
            case MIDDLE_RETRACT_ELBOW_S1:
                pos_elbow = elbow.getPosition();

                // stay until the elbow is within threshold
                if (bucketStateTimer.seconds() > LAPSE_SEC_ELBOW_S1) {
                    elbow.setElbow(0);
                    bucketStateTimer.reset();
                    bucketState = BucketState.MIDDLE_RETRACT_ELBOW_S2;
                }
                break;

            case MIDDLE_RETRACT_ELBOW_S2:
                pos_elbow = elbow.getPosition();

                // stay until the elbow is within threshold
                if (bucketStateTimer.seconds() > 1 ||
                        ((pos_elbow >= 0 - THRESH_ELBOW)
                        && (pos_elbow <= 0 +THRESH_ELBOW))) {
                    viper.setTarget(0);
                    bucketStateTimer.reset();
                    bucketState = BucketState.MIDDLE_RETRACT_VIPER;
                }
                break;

            case MIDDLE_RETRACT_VIPER:
                pos_viper = viper.getPosition();
                if (bucketStateTimer.seconds() > 1 || pos_viper <= +THRESH_VIPER) {
                    bucketStateTimer.reset();
                    shoulder.setTarget(POS_SHOULDER_MIDDLE_BUCKET);
                    bucketState = BucketState.MIDDLE_BUCKETSTATE;
                }
                break;

            case HIGH_EXTEND_VIPER:
                pos_viper = viper.getPosition();

                // stay until the shoulder is within threshold
                if ((pos_viper >= POS_VIPER_HIGH_BUCKET-THRESH_VIPER)
                        && (pos_viper <= POS_VIPER_HIGH_BUCKET+THRESH_VIPER)) {

                    // extend the elbow
                    elbow.setElbow(POS_ELBOW_EXTEND_HIGH_BUCKET);
                    bucketState = BucketState.HIGH_EXTEND_ELBOW;
                }
                break;

            case HIGH_EXTEND_ELBOW:
                pos_elbow = elbow.getPosition();

                // stay until the elbow is within threshold
                if ((pos_elbow >= POS_ELBOW_EXTEND_HIGH_BUCKET -THRESH_ELBOW)
                        && (pos_elbow <= POS_ELBOW_EXTEND_HIGH_BUCKET +THRESH_ELBOW)) {

                    // listen to the command to retract
                    if (gamepad.b) {
                        elbow.setElbow(POS_ELBOW_REST);
                        bucketStateTimer.reset();
                        bucketState = BucketState.HIGH_RETRACT_ELBOW_S1;
                    }
                }
                break;

            case HIGH_RETRACT_ELBOW_S1:
                pos_elbow = elbow.getPosition();

                // stay until the elbow is within threshold
                if (bucketStateTimer.seconds() > LAPSE_SEC_ELBOW_S1) {
                    elbow.setElbow(0);
                    bucketStateTimer.reset();
                    bucketState = BucketState.HIGH_RETRACT_ELBOW_S2;
                }
                break;

            case HIGH_RETRACT_ELBOW_S2:
                pos_elbow = elbow.getPosition();

                // stay until the elbow is within threshold
                if (bucketStateTimer.seconds() > 1
                        || ((pos_elbow >= 0 - THRESH_ELBOW)
                        && (pos_elbow <= 0 +THRESH_ELBOW))) {
                    viper.setTarget(0);
                    bucketStateTimer.reset();
                    bucketState = BucketState.HIGH_RETRACT_VIPER;
                }
                break;

            case HIGH_RETRACT_VIPER:
                pos_viper = viper.getPosition();
                if (bucketStateTimer.seconds() > 1 || pos_viper <= +THRESH_VIPER) {
                    shoulder.setTarget(POS_SHOULDER_HIGH_BUCKET);
                    bucketStateTimer.reset();
                    bucketState = BucketState.HIGH_BUCKETSTATE;
                }
                break;
        }
    }

}