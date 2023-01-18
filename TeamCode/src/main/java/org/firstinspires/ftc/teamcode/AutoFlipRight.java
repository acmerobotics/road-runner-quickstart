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
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Flip Right", group="Concept")
//@Disabled
public class AutoFlipRight extends AutonomousRight {

    @Override
    public void autonomousCore() {
        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
        armClaw.armFlipCenter(); // armFlipBackUnload();
        //move center of robot to the edge of 3rd mat
        chassis.drivingWithSensor(-Params.INIT_POSITION_TO_2ND_MAT_EDGE, false,
                chassis.frontCenterDS, 12, true, true);

        // turn robot to make sure it is at 0 degree before backing to mat center
        chassis.rotateIMUTargetAngle(0);

        // lift slider
        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
        slider.waitRunningComplete();

        //drive forward and let V touch junction
        chassis.drivingWithSensor(-Params.CHASSIS_HALF_WIDTH, true,
                chassis.backCenterDS, Params.UNLOAD_DS_VALUE, false, true);
        Logging.log("Autonomous - Robot V reached junction.");

        // drop cone and back to the center of mat
        unloadCone();

        for(int autoLoop = 0; autoLoop < 3; autoLoop++) {
            Logging.log("Autonomous - loop index: %d ", autoLoop);

            // drive robot to cone stack.
            moveToConeStack();

            // load cone
            loadCone(Params.coneStack5th - Params.coneLoadStackGap * autoLoop);

            // drive robot to junction
            moveToMediumJunction();

            Logging.log("Autonomous - slider is positioned to medium junction.");

            // unload cone
            unloadCone();
            Logging.log("Autonomous - cone %d has been unloaded.", autoLoop + 2);
        }

        // lower slider in prep for tele-op
        slider.setInchPosition(Params.GROUND_CONE_POSITION);
        armClaw.armFlipCenter();

        // move to center of 3rd mat
        chassis.runToPosition(-Params.HALF_MAT * autonomousStartLocation, false);

        // drive to final parking lot
        chassis.runToPosition(parkingLotDis * autonomousStartLocation, true);
        Logging.log("Autonomous - Arrived at parking lot Mat: %.2f", parkingLotDis);

        slider.waitRunningComplete();
        Logging.log("Autonomous - Autonomous complete.");
    }


    /**
     * unload cone
     */
    private void unloadCone() {
        armClaw.armFlipCenter(); //armFlipBackUnload();
        armClaw.clawOpen();
        // Make sure it is 45 degree before V leaving junction
        chassis.rotateIMUTargetAngle(0);
        sleep(100); // to make sure claw Servo is at open position, 250 ms
        armClaw.armFlipCenter(); //armFlipFrontLoad();
        Logging.log("Auto unload - Cone has been unloaded.");
    }

    /**
     * During autonomous, cone may be located with different height position
     * @param coneLocation: the target cone high location in inch.
     */
    private void loadCone(double coneLocation) {
        slider.setInchPosition(coneLocation);
        chassis.runToPosition(-Params.DISTANCE_PICK_UP, true); // moving to loading position
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(100); // wait to make sure clawServo is at grep position, 200 ms
        chassis.rotateIMUTargetAngle(0);
        slider.setInchPosition(Params.WALL_POSITION);
        armClaw.armFlipCenter(); //armFlipBackUnload();
        slider.waitRunningComplete(); // make sure slider has been lifted.
    }

    /**
     * move to cone stack and lower sliders
     */
    private void moveToConeStack() {
        while (chassis.getEncoderDistance() < Params.HALF_MAT * 2) {
            chassis.drivingWithPID(chassis.AUTO_MAX_POWER, 0.0, -chassis.AUTO_MAX_POWER, true);
        }

        slider.setInchPosition(Params.WALL_POSITION);

        chassis.drivingWithSensor(3 * Params.HALF_MAT - Params.CHASSIS_LENGTH / 2, true,
                chassis.frontCenterDS, Params.LOAD_DS_VALUE, false, true);

        slider.waitRunningComplete();
    }

    /**
     * move to junction and lift sliders
     */
    private void moveToMediumJunction() {
        chassis.drivingWithSensor(-Params.HALF_MAT, true,
                chassis.backCenterDS, 0, true, false);

        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);

        while (chassis.getEncoderDistance() < Params.HALF_MAT * 2) {
            chassis.drivingWithPID(-chassis.AUTO_MAX_POWER, 0.0, -chassis.AUTO_MAX_POWER, true);
        }

        chassis.drivingWithSensor(-Params.HALF_MAT, true,
                chassis.backCenterDS, Params.UNLOAD_DS_VALUE, false, true);
        slider.waitRunningComplete();
    }

/**
 * Set robot starting position: 1 for right and -1 for left.
 */
    public void setRobotLocation() {
        autonomousStartLocation = 1;
    }

}