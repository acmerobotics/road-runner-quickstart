/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
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

/*
 * PID controller and IMU codes are copied from
 * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Extended from AutonomousRight file.
 * Use this one for autonomous when robot located at left side of game field.
 */

@Autonomous(name="Only For Test", group="Concept")
@Disabled
public class AutonomousTest extends AutonomousRight {

    @Override
    public void autonomousCore() {

        // lift slider
        slider.setInchPosition(Params.LOW_JUNCTION_POS);

        //move center of robot to the edge of 3rd mat
        chassis.drivingWithSensor(Params.INIT_POSITION_TO_2ND_MAT_EDGE, false,
                chassis.frontCenterDS, 12, true, true);

        // turn robot to make sure it is at 0 degree before backing to mat center
        chassis.rotateIMUTargetAngle(0);


        // lift slider during rotating robot 45 degrees left
        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);

        // driving back to mat center
        chassis.drivingWithSensor(-Params.CHASSIS_HALF_WIDTH + Params.BACK_V_TO_CENTER, true,
                chassis.backCenterDS, Params.UNLOAD_DS_VALUE, true, true);

        // drop cone and back to the center of mat
        armClaw.armFlipBackUnload();
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION);
        slider.waitRunningComplete();
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // 200
        chassis.rotateIMUTargetAngle(0);
        armClaw.armFlipFrontLoad();
        chassis.runToPosition(Params.CHASSIS_HALF_WIDTH - Params.BACK_V_TO_CENTER, true); // move out from junction
        slider.setInchPosition(Params.WALL_POSITION);
        Logging.log("Auto unload - Cone has been unloaded.");

        // driving back to mat center

        chassis.runToPosition(-Params.CHASSIS_HALF_WIDTH, false);

        for(int autoLoop = 0; autoLoop < 3; autoLoop++) {

            // right turn 45 degree.
            chassis.rotateIMUTargetAngle(0.0 * autonomousStartLocation);

            // Rotation for accurate 45 degrees
            chassis.rotateIMUTargetAngle(0.0 * autonomousStartLocation);

            // drive robot to cone loading area.
            //chassis.runToPosition(movingToConeStack, true);
            chassis.drivingWithSensor(movingToConeStack, true,
                    chassis.frontCenterDS, Params.LOAD_DS_VALUE, true, true);

            // load cone
            autoLoadCone(Params.coneStack5th - Params.coneLoadStackGap * autoLoop);

            //chassis.runToPosition(-MovingToMatCenter, true);
            chassis.drivingWithSensor(-MovingToMatCenter, true,
                    chassis.backCenterDS, Params.UNLOAD_DS_VALUE, true, true);

            // lift slider during left turning 45 degree facing to junction.
            slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
            chassis.rotateIMUTargetAngle(45.0 * autonomousStartLocation);

            //Rotation for accurate 45 degrees
            //chassis.rotateIMUTargetAngle(-45.0 * autonomousStartLocation);

            // moving forward V to junction
            chassis.runToPosition(-movingDistBeforeDrop, true);

            // unload cone & adjust, 0.2 inch for cone thickness adjust
            autoUnloadCone(movingDistAfterDrop);
        }

        //rotate 45 degrees to keep orientation at 90
        chassis.rotateIMUTargetAngle(0.0 * autonomousStartLocation);

        // lower slider in prep for tele-op
        slider.setInchPosition(Params.GROUND_CONE_POSITION);
        armClaw.armFlipFrontLoad();

        // drive to final parking lot, -1 for arm extended out of chassis.
        chassis.runToPosition(parkingLotDis * autonomousStartLocation - 1, true);
        Logging.log("Autonomous - Arrived at parking lot Mat: %.2f", parkingLotDis);

        slider.waitRunningComplete();
        Logging.log("Autonomous - Autonomous complete.");
    }


    /**
     * During autonomous, cone may be located with different height position
     */
    private void loadCone() {
        slider.setInchPosition(Params.coneStack5th);
        chassis.runToPosition(-Params.DISTANCE_PICK_UP, true); // moving to loading position
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(100); // wait to make sure clawServo is at grep position, 200 ms
        chassis.rotateIMUTargetAngle(0);
        slider.setInchPosition(Params.WALL_POSITION);
        armClaw.armFlipBackUnload();
        slider.waitRunningComplete(); // make sure slider has been lifted.
    }
}
