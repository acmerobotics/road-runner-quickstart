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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

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

@Autonomous(name="Autonomous_Right", group="Concept")
//@Disabled
public class AutonomousRight extends LinearOpMode {

    public int autonomousStartLocation = 1; // 1 for right location, and -1 for left location.

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    public final ChassisWith4Motors chassis = new ChassisWith4Motors();
    public final SlidersWith2Motors slider = new SlidersWith2Motors();
    public final ArmClawUnit armClaw = new ArmClawUnit();

    // variables for autonomous
    double matCenterToJunction = Params.HALF_MAT * 1.414 - Params.BACK_V_TO_CENTER - Params.CONE_WALL_THICKNESS; //12.468
    double matCenterToConeStack = Params.HALF_MAT * 3 - Params.FLIP_ARM_LENGTH; // 25.5

    //  adjust -2 ~ 2 inch if needed for below 5 variables
    double backDistanceToThrowSleeve = Params.HALF_MAT - Params.CHASSIS_LENGTH / 2.0 + 1;
    double movingDistBeforeDrop = matCenterToJunction;
    double movingDistAfterDrop = matCenterToJunction - 1;
    double movingToConeStack = matCenterToConeStack - 1.0;
    double MovingToMatCenter = matCenterToConeStack - Params.DISTANCE_PICK_UP - 3;

    // camera and sleeve color
    ObjectDetection.ParkingLot myParkingLot = ObjectDetection.ParkingLot.UNKNOWN;
    double parkingLotDis = 0;
    ObjectDetection coneSleeveDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setRobotLocation();

        // camera for sleeve color detect, start camera at the beginning.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        coneSleeveDetect = new ObjectDetection();

        if (isCameraInstalled) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                    WebcamName.class, webcamName), cameraMonitorViewId);

            camera.setPipeline(coneSleeveDetect);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    Logging.log("Start stream to detect sleeve color.");
                    telemetry.addData("Start stream to detect sleeve color.", "ok");
                    telemetry.update();
                }

                @Override
                public void onError(int errorCode) {
                    Logging.log("Start stream error.");
                    telemetry.addData("Start stream to detect sleeve color.", "error");
                    telemetry.update();
                }
            });
        }

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slider.init(hardwareMap, "RightSlider", "LeftSlider");
        // Reset slider motor encoder counts kept by the motor
        slider.resetEncoders();

        chassis.init(hardwareMap, "FrontLeft", "FrontRight",
                "BackLeft", "BackRight");

        armClaw.init(hardwareMap, "ArmServo", "ClawServo");
        armClaw.armFlipFrontLoad();
        sleep(200);
        armClaw.clawClose();
        sleep(300);
        armClaw.armFlipCenter();

        runtime.reset();
        while ((ObjectDetection.ParkingLot.UNKNOWN == myParkingLot) &&
                ((runtime.seconds()) < 3.0)) {
            myParkingLot = coneSleeveDetect.getParkingLot();
        }
        Logging.log("Parking Lot position: %s", myParkingLot.toString());

        while (!isStarted()) {
            myParkingLot = coneSleeveDetect.getParkingLot();
            parkingLotDis = coneSleeveDetect.getParkingLotDistance();
            telemetry.addData("Parking position: ", myParkingLot);
            telemetry.addData("robot position: ", autonomousStartLocation > 0? "Right":"Left");
            telemetry.addData("Front Center distance sensor", "%.2f", chassis.getFcDsValue());
            telemetry.addData("Back center distance sensor", "%.2f", chassis.getBcDsValue());
            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            camera.stopRecordingPipeline();
            camera.closeCameraDevice();

            autonomousCore();

            Logging.log("Autonomous - total Run Time: " + runtime);
        }
        // The motor stop on their own but power is still applied. Turn off motor.
        slider.stop();
        chassis.setPowers(0.0);
    }

    /** code for autonomous
     * 1. take a picture, recognize the color on sleeve signal
     * 2. Move robot the high junction
     * 3. Unload cone on high junction
     * 4. Move robot to cone loading area
     * 5. Load cone
     * 6. Move robot to parking area
     */
    public void autonomousCore() {

        // lift slider
        slider.setInchPosition(Params.LOW_JUNCTION_POS);

        //move center of robot to the edge of 3rd mat
        chassis.runToPosition(6 * Params.HALF_MAT - Params.CHASSIS_LENGTH, true);

        sleep(100);
        // turn robot to make sure it is at 0 degree before backing to mat center
        chassis.rotateIMUTargetAngle(0.0);

        // lift slider during rotating robot 45 degrees left
        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);

        // driving back to mat center
        chassis.runToPosition(-backDistanceToThrowSleeve, true);

        chassis.rotateIMUTargetAngle(-45.0 * autonomousStartLocation);
        armClaw.armFlipBackUnload();

        //drive forward and let V to touch junction
        chassis.runToPosition(-movingDistBeforeDrop, true);
        
        // drop cone and back to the center of mat
        autoUnloadCone(movingDistAfterDrop + Params.CONE_WALL_THICKNESS * 3);

        for(int autoLoop = 0; autoLoop < 3; autoLoop++) {

            // right turn 45 degree.
            chassis.rotateIMUTargetAngle(-90.0 * autonomousStartLocation);

            // Rotation for accurate 45 degrees
            chassis.rotateIMUTargetAngle(-90.0 * autonomousStartLocation);

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
            chassis.rotateIMUTargetAngle(-45.0 * autonomousStartLocation);

            //Rotation for accurate 45 degrees
            //chassis.rotateIMUTargetAngle(-45.0 * autonomousStartLocation);

            // moving forward V to junction
            chassis.runToPosition(-movingDistBeforeDrop, true);

            // unload cone & adjust, 0.2 inch for cone thickness adjust
            autoUnloadCone(movingDistAfterDrop);

            //moving slider a little bit to save time during picking up cone
            if (autoLoop > 0) {
                slider.setInchPosition(Params.WALL_POSITION - Params.coneLoadStackGap * (autoLoop - 1));
            }
        }

        //rotate 45 degrees to keep orientation at 90
        chassis.rotateIMUTargetAngle(-90.0 * autonomousStartLocation);

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
     * @param coneLocation: the target cone high location.
     */
    public void autoLoadCone(double coneLocation) {
        slider.setInchPosition(coneLocation);
        chassis.runToPosition(-Params.DISTANCE_PICK_UP, true); // moving to loading position
        Logging.log("Wait before claw close.");
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(Params.CLAW_CLOSE_SLEEP - 50);// subtract 50ms due to the following rotation function.
        chassis.rotateIMUTargetAngle(-90.0 * autonomousStartLocation);
        slider.movingSliderInch(Params.SLIDER_MOVE_OUT_CONE_STACK);
        armClaw.armFlipBackUnload();
        Logging.log("Wait before moving out cone stack.");
        slider.waitRunningComplete(); // make sure slider has been lifted.
    }

    /**
     * @param moveDistanceAfterDrop moving back distance after dropping the cone to move out claw from junction.
     */
    public void autoUnloadCone(double moveDistanceAfterDrop) {
        armClaw.armFlipBackUnload();
        chassis.runToPosition(Params.DISTANCE_DROP_OFF, true);
        sleep(Params.WAIT_SHAKING_SLEEP);
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION);
        Logging.log("Wait before unloading claw open.");
        slider.waitRunningComplete();
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP - 100); // 200
        chassis.rotateIMUTargetAngle(-45.0 * autonomousStartLocation);
        armClaw.armFlipFrontLoad();
        chassis.runToPosition(moveDistanceAfterDrop - Params.DISTANCE_DROP_OFF, true); // move out from junction
        slider.setInchPosition(Params.WALL_POSITION);
        Logging.log("Auto unload - Cone has been unloaded.");
    }

/**
 * Set robot starting position: 1 for right and -1 for left.
 */
    public void setRobotLocation() {
        autonomousStartLocation = 1;
    }

}