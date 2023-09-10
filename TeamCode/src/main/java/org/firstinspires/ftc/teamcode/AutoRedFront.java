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

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.with2DW;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.with3DW;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 * Hardware config:
 *      imu on control Hub:
 *          "imu"
 *
 *      Four drive motors:
 *          "FrontLeft"
 *          "BackLeft"
 *          "BackRight"
 *          "FrontRight"
 *
 *      One servo motors:
 *          "ArmServo"
 *          "ClawServo"
 *
 *      Two cameras:
 *          "Webcam 1"
 *          "WebcamR"
 */

@Autonomous(name="Auto Red Front", group="Concept")
//@Disabled
public class AutoRedFront extends LinearOpMode {

    // average calibration values - good for the match in BC
    Pose2d preConeDropAdjust = new Pose2d(0, 0, 0);
    Pose2d poseConeStackAdjust = new Pose2d(0, 0, 0);
    Pose2d poseMJDropOffAdjust = new Pose2d(0, 0, 0);

    boolean withDW = with2DW || with3DW; // with dead wheels
    boolean compensationOn = true;

    // 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
    public int startLoc = 2;
    public int sparkMarkLoc = 1; // 1 for left, 2 for center, and 3 for right
    boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ArmClawUnit armClaw = new ArmClawUnit();

    public SampleMecanumDrive drive;

    // camera and sleeve color
    ObjectDetection.PropSide myParkingLot = ObjectDetection.PropSide.UNKNOWN;

    ObjectDetection coneSleeveDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;

    // road runner variables
    Pose2d startPose;

    // pre cone to medium junction
    Pose2d poseLineEnd1, poseRedBackDropCenter;

    Trajectory traj1;
    TrajectorySequence trajSeq1; // use circle path
    TrajectorySequence trajSeq2;

    /**
     * Set robot starting position: 1 for right and -1 for left.
     */
    public void setRobotLocation() {
        startLoc = 1;

        // compensationOn parameters
        if (!withDW) {
            preConeDropAdjust = new Pose2d(0, 0, 0);
            poseConeStackAdjust = new Pose2d(0, 0, 0);
            poseMJDropOffAdjust = new Pose2d(0, 0, 0);
        }
        else {
            preConeDropAdjust = new Pose2d(-0.5, -0.5, 0);
            poseConeStackAdjust = new Pose2d(-0.5, 0.5, 0);
            poseMJDropOffAdjust = new Pose2d(-0.5, 0, 0);
        }
        Logging.log("dead wheel on? %s.", withDW? "yes" : "No");
        Logging.log("Compensation is %s.", compensationOn? "On" : "Off");
    }

    private void setPoses() {
        // road runner variables
        if (1 == startLoc) { // red front
            startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2.0,
                    3 * Params.HALF_MAT, Math.toRadians(0));
        }

        if (2 == startLoc) { // red back
            startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2.0,
                    -1 * Params.HALF_MAT, Math.toRadians(0));
        }

        if (3 == startLoc) { //  blue front
            startPose = new Pose2d(6 * Params.HALF_MAT - Params.CHASSIS_LENGTH / 2.0,
                    3 * Params.HALF_MAT, Math.toRadians(180));
        }

        if (4 == startLoc) { //  blue back
            startPose = new Pose2d(6 * Params.HALF_MAT - Params.CHASSIS_LENGTH / 2.0,
                    -1 * Params.HALF_MAT, Math.toRadians(180));
        }
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setRobotLocation();

        setPoses();

        // camera for sleeve color detect, start camera at the beginning.
        if (2 == startLoc) webcamName = "Webcam 1";
        else webcamName = "WebcamR";
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

        // init drive with road runner
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        Params.currentPose = startPose; // init storage pose.

        armClaw.init(hardwareMap, "ArmMotor", "ClawServo");

        armClaw.resetArmEncoder();

        sleep(500);
        armClaw.clawClose();
        sleep(500);
        armClaw.armManualMoving(10);

        runtime.reset();
        while ((ObjectDetection.PropSide.UNKNOWN == myParkingLot) &&
                ((runtime.seconds()) < 3.0)) myParkingLot = coneSleeveDetect.getPropPos();
        Logging.log("Parking Lot position: %s", myParkingLot.toString());

        while (!isStarted()) {
            myParkingLot = coneSleeveDetect.getPropPos();
            telemetry.addData("Parking position: ", myParkingLot);
            telemetry.addData("robot position: ", startLoc > 0? "Right":"Left");
            telemetry.addData("RR", "imu Heading = %.1f",
                    Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);


        switch (myParkingLot) {
            case LEFT:
                sparkMarkLoc = 1;
                break;
            case CENTER:
            case UNKNOWN:
                sparkMarkLoc = 2;
                break;
                case RIGHT:
                sparkMarkLoc = 3;
                break;
        }

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            autonomousCore();
            camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
            Logging.log("Autonomous time - total Run Time: " + runtime);
        }
    }

    public void autonomousCore() {

        switch (startLoc) {
            case 1:
                autoRedFrontCore();
                break;
            case 2:
                autoRedBackCore();
                break;
            case 3:
                autoBlueFrontCore();
                break;
            case 4:
                autoBlueBackCore();
                break;
        }
    }

    // 1 == startLoc
    private void autoRedFrontCore() {
        // 1. move to central line
        poseLineEnd1 = new Pose2d(-3 * Params.HALF_MAT, 3 * Params.HALF_MAT, startPose.getHeading());
        poseRedBackDropCenter = new Pose2d(-3 * Params.HALF_MAT, -4 * Params.HALF_MAT, Math.toRadians(-90.0));
        Pose2d poseRedBackDropRight = new Pose2d(poseRedBackDropCenter.getX() - Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));
        Pose2d poseRedBackDropLeft = new Pose2d(poseRedBackDropCenter.getX() + Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));

        traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseLineEnd1)
                .build();
        drive.followTrajectory(traj1);

        if (1 == sparkMarkLoc) {// left

            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);

            // 4. turn 180
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(180.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseRedBackDropCenter)
                    .addDisplacementMarker(4*Params.HALF_MAT, () -> {
                        armClaw.armLift();
                    })
                    .lineToLinearHeading(poseRedBackDropLeft)
                    .build();
            drive.followTrajectory(traj1);
            armClaw.clawOpen();
        }

        if (2 == sparkMarkLoc) { // center

            // 2. open claw, to release the purple pixel
            armClaw.clawOpen();
            sleep(100);

            // 3. close claw to pick-up the yellow pixel
            armClaw.clawClose();
            sleep(100);
            // 3. turn right 90 degree and move to backdrop
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(-90.0))
                    .lineToLinearHeading(poseRedBackDropCenter)
                    .addDisplacementMarker(4*Params.HALF_MAT, () -> {
                        armClaw.armLift();
                    })
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. open claw to release the pixel.
            armClaw.clawOpen();
        }

        if (3 == sparkMarkLoc) // right
        {
            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(-90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);

            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseRedBackDropCenter)
                    .addDisplacementMarker(4*Params.HALF_MAT, () -> {
                        armClaw.armLift();
                    })
                    .lineToLinearHeading(poseRedBackDropRight)
                    .build();
            drive.followTrajectory(traj1);
            armClaw.clawOpen();
        }
    }
    private void autoRedBackCore() {
        // 2 == startLoc
        // 1. move to central line
        poseLineEnd1 = new Pose2d(-3 * Params.HALF_MAT, -1 * Params.HALF_MAT, startPose.getHeading());
        poseRedBackDropCenter = new Pose2d(-3 * Params.HALF_MAT, -4 * Params.HALF_MAT, Math.toRadians(-90.0));
        Pose2d poseRedBackDropRight = new Pose2d(poseRedBackDropCenter.getX() - Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));
        Pose2d poseRedBackDropLeft = new Pose2d(poseRedBackDropCenter.getX() + Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));


        traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseLineEnd1)
                .build();
        drive.followTrajectory(traj1);

        if (1 == sparkMarkLoc) {// left

            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);
            armClaw.armLift();

            // 4. turn 180
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(180.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseRedBackDropCenter)
                    .lineToLinearHeading(poseRedBackDropLeft)
                    .build();
            drive.followTrajectory(traj1);
            armClaw.clawOpen();
        }

        if (2 == sparkMarkLoc) { // center

            // 2. open claw, to release the purple pixel
            armClaw.clawOpen();
            sleep(100);

            // 3. close claw to pick-up the yellow pixel
            armClaw.clawClose();
            sleep(100);
            armClaw.armLift();

            // 3. turn right 90 degree and move to backdrop
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(-90.0))
                    .lineToLinearHeading(poseRedBackDropCenter)
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. open claw to release the pixel.
            armClaw.clawOpen();
        }

        if (3 == sparkMarkLoc) // right
        {
            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(-90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);
            armClaw.armLift();

            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseRedBackDropCenter)
                    .lineToLinearHeading(poseRedBackDropRight)
                    .build();
            drive.followTrajectory(traj1);
            armClaw.clawOpen();
        }

        if (debug_flag) {
            Logging.log("Autonomous time- ending traj1: " + runtime);

            Logging.log("Arrived pre cone junction");
            Logging.log("start pose: end x = %.2f,  y = %.2f, angle = %.2f",
                    startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));

            Logging.log("pre-cone pose trajSeq1: end x = %.2f,  y = %.2f, angle = %.2f",
                    trajSeq1.end().getX(), trajSeq1.end().getY(), Math.toDegrees(trajSeq1.end().getHeading()));

            Logging.log("pre-cone pose  trajSeq2: end x = %.2f,  y = %.2f, angle = %.2f",
                    trajSeq2.end().getX(), trajSeq2.end().getY(), Math.toDegrees(trajSeq2.end().getHeading()));

            Logging.log("pre-cone pose traj1: end x = %.2f,  y = %.2f, angle = %.2f",
                    traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));

            Logging.log("pre-cone pose estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
        }

    }

    // 3 = startLoc
    private void autoBlueFrontCore(){
        // 1. move to central line
        Pose2d poseLineEnd1 = new Pose2d(3 * Params.HALF_MAT, 3 * Params.HALF_MAT, startPose.getHeading());
        Pose2d poseBlueBackDropCenter = new Pose2d(3 * Params.HALF_MAT, -4 * Params.HALF_MAT, Math.toRadians(-90.0));
        Pose2d poseBlueBackDropRight = new Pose2d(poseRedBackDropCenter.getX() - Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));
        Pose2d poseBlueBackDropLeft = new Pose2d(poseRedBackDropCenter.getX() + Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));

        traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseLineEnd1)
                .build();
        drive.followTrajectory(traj1);

        if (3 == sparkMarkLoc) {// right

            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(-90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);

            // 4. turn 180
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(180.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseBlueBackDropCenter)
                    .addDisplacementMarker(4*Params.HALF_MAT, () -> {
                        armClaw.armLift();
                    })
                    .lineToLinearHeading(poseBlueBackDropLeft)
                    .build();
            drive.followTrajectory(traj1);
            armClaw.clawOpen();
        }

        if (2 == sparkMarkLoc) { // center

            // 2. open claw, to release the purple pixel
            armClaw.clawOpen();
            sleep(100);

            // 3. close claw to pick-up the yellow pixel
            armClaw.clawClose();
            sleep(100);

            // 3. turn right 90 degree and move to backdrop
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(90.0))
                    .addDisplacementMarker(4*Params.HALF_MAT, () -> {
                        armClaw.armLift();
                    })
                    .lineToLinearHeading(poseBlueBackDropCenter)
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. open claw to release the pixel.
            armClaw.clawOpen();
        }

        if (1 == sparkMarkLoc) // left
        {
            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);

            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseBlueBackDropCenter)
                    .addDisplacementMarker(4*Params.HALF_MAT, () -> {
                        armClaw.armLift();
                    })
                    .lineToLinearHeading(poseBlueBackDropRight)
                    .build();
            drive.followTrajectory(traj1);
            armClaw.clawOpen();
        }
    }

    private void autoBlueBackCore() {
        // startLoc = 4
        // 1. move to central line
        Pose2d poseLineEnd1 = new Pose2d(3 * Params.HALF_MAT, -1 * Params.HALF_MAT, startPose.getHeading());
        Pose2d poseBlueBackDropCenter = new Pose2d(3 * Params.HALF_MAT, -4 * Params.HALF_MAT, Math.toRadians(-90.0));
        Pose2d poseBlueBackDropRight = new Pose2d(poseRedBackDropCenter.getX() - Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));
        Pose2d poseBlueBackDropLeft = new Pose2d(poseRedBackDropCenter.getX() + Params.HALF_MAT, poseRedBackDropCenter.getY(), Math.toRadians(-90.0));

        traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseLineEnd1)
                .build();
        drive.followTrajectory(traj1);

        if (3 == sparkMarkLoc) {// right

            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(-90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);
            armClaw.armLift();

            // 4. turn 180
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(180.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseBlueBackDropCenter)
                    .lineToLinearHeading(poseBlueBackDropLeft)
                    .build();
            drive.followTrajectory(traj1);
            armClaw.clawOpen();
        }

        if (2 == sparkMarkLoc) { // center

            // 2. open claw, to release the purple pixel
            armClaw.clawOpen();
            sleep(100);

            // 3. close claw to pick-up the yellow pixel
            armClaw.clawClose();
            sleep(100);
            armClaw.armLift();

            // 3. turn right 90 degree and move to backdrop
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(90.0))
                    .lineToLinearHeading(poseBlueBackDropCenter)
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. open claw to release the pixel.
            armClaw.clawOpen();
        }

        if (1 == sparkMarkLoc) // left
        {
            // 1. turn -90
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(90.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);
            armClaw.armLift();
            // 4. move to Backdrop
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseBlueBackDropCenter)
                    .lineToLinearHeading(poseBlueBackDropRight)
                    .build();
            drive.followTrajectory(traj1);

            armClaw.clawOpen();
        }
    }
}
