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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 * Extended from AutonomousRight file.
 * Use this one for autonomous when robot located at left side of game field.
 */

@Autonomous(name="auto RR right", group="Concept")
//@Disabled
public class AutoRoadRunner extends LinearOpMode {

    public int autonomousStartLocation = 1; // 1 for right location, and -1 for left location.

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    public final SlidersWith2Motors slider = new SlidersWith2Motors();
    public final ArmClawUnit armClaw = new ArmClawUnit();

    // init drive with road runner
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

    // road runner variables
    Pose2d startPose = new Pose2d(0,0,0);


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
            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Wait for the game to start (driver presses PLAY)
        drive.setPoseEstimate(new Pose2d());

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
    }

    ////////////////////////////////////////////////////////////////////////////////////////
    public void autonomousCore() {

        // lift slider
        slider.setInchPosition(Params.LOW_JUNCTION_POS);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(6 * Params.HALF_MAT - Params.CHASSIS_LENGTH)
                .build();
        drive.followTrajectory(traj1);

        // lift slider during rotating robot 45 degrees left
        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
        armClaw.armFlipBackUnload();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() - Params.CHASSIS_HALF_LENGTH + Params.HALF_MAT * 4 + Params.CHASSIS_HALF_LENGTH / 1.414,
                        startPose.getY() - Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH / 1.414,
                        startPose.getHeading() + Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj2);

        // drop cone and back to the center of mat
        rrUnloadCone();

        for(int autoLoop = 0; autoLoop < 1; autoLoop++) {


            moveFromJunctionToConeStack();

            // load cone
            rrLoadCone(Params.coneStack5th - Params.coneLoadStackGap * autoLoop);

            moveFromConeStackToJunction();

            // unload cone & adjust, 0.2 inch for cone thickness adjust
            rrUnloadCone();
        }

        //rotate 45 degrees to keep orientation at 90
        //chassis.rotateIMUTargetAngle(0.0 * autonomousStartLocation);

        // lower slider in prep for tele-op
        slider.setInchPosition(Params.GROUND_CONE_POSITION);
        armClaw.armFlipFrontLoad();

        // drive to final parking lot, -1 for arm extended out of chassis.
        //chassis.runToPosition(parkingLotDis * autonomousStartLocation - 1, true);
        Logging.log("Autonomous - Arrived at parking lot Mat: %.2f", parkingLotDis);

        slider.waitRunningComplete();
        Logging.log("Autonomous - Autonomous complete.");
    }


    /**
     * During autonomous, cone may be located with different height position
     * @param coneLocation: the target cone high location.
     */
    public void rrLoadCone(double coneLocation) {
        slider.setInchPosition(coneLocation);
        driveBack(Params.DISTANCE_PICK_UP);
        Logging.log("Wait before claw close.");
        slider.waitRunningComplete();
        armClaw.clawClose();
        sleep(Params.CLAW_CLOSE_SLEEP);// subtract 50ms due to the following rotation function.
        slider.movingSliderInch(Params.SLIDER_MOVE_OUT_CONE_STACK);
        armClaw.armFlipBackUnload();
        Logging.log("Wait before moving out cone stack.");
        slider.waitRunningComplete(); // make sure slider has been lifted.
    }

    /**
     * auto unload cone with roadrunner support.
     */
    public void rrUnloadCone() {
        armClaw.armFlipBackUnload();
        driveForward(Params.DISTANCE_DROP_OFF);
        sleep(Params.WAIT_SHAKING_SLEEP);
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION);
        Logging.log("Wait before unloading claw open.");
        slider.waitRunningComplete();
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // 200
        armClaw.armFlipFrontLoad();

        Logging.log("Auto unload - Cone has been unloaded.");
    }

    private void moveFromJunctionToConeStack() {

    }

    private void moveFromConeStackToJunction() {

    }

    private void driveForward(double distance) {
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(distance)
                .build();
        drive.followTrajectory(traj);
    }

    private void driveBack(double distance) {
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(distance)
                .build();
        drive.followTrajectory(traj);
    }

    /**
     * Set robot starting position: 1 for right and -1 for left.
     */
    public void setRobotLocation() {
        autonomousStartLocation = 1;
    }
}
