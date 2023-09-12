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

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;

/**
 * This is NOT an opmode.
 *
 * This class defines the parameters related with game field elements.
 */
public class Params {
    // road runner
    public static Pose2d currentPose = new Pose2d(0,0,0);

    //cone parameters
    static final double coneLoadStackGap = 1.3;

    // autonomous driving distance
    static final double HALF_MAT = 12.0;
    static final double CHASSIS_HALF_WIDTH = 14 / 2.0;
    static final double CHASSIS_LENGTH = 17;
    static final double FLIP_ARM_LENGTH = 10;
    static final double BASE_TO_JUNCTION = 3 * HALF_MAT - 18;
    static final double UNLOAD_DS_VALUE = 4.5;

    // moving distance variables
    static final double DISTANCE_PICK_UP = 1.0; // in INCH
    static final double GAP_DISTANCE = (HALF_MAT - CHASSIS_LENGTH / 2.0) / 2.0;

    // slider position variables
    static final double ARM_UNLOADING_LIFTING = 4.0;
    static final double ARM_UNLOADING_EXTENSION = 7.0;
    static final double GROUND_CONE_POSITION = 0.0;
    static final double coneStack5th = coneLoadStackGap * 4;
    static final double LOW_JUNCTION_POS = 13.5 - ARM_UNLOADING_LIFTING;
    static final double MEDIUM_JUNCTION_POS = 23.5 - ARM_UNLOADING_LIFTING;
    static final double HIGH_JUNCTION_POS = 33.5 - ARM_UNLOADING_LIFTING;
    static final double WALL_POSITION = 9.1;
    static final double GROUND_CONE_READY_POSITION = 4.8;
    static final double SLIDER_MOVE_DOWN_POSITION = 4.0;

    //claw action time
    static final int CLAW_CLOSE_SLEEP = 150; // ms
    static final int CLAW_OPEN_SLEEP = 50; // ms

    // chassis power factors
    static final double POWER_LOW = 0.3;
    static final double POWER_NORMAL = 0.75;
    static final double POWER_HIGH = 1.0;
}

