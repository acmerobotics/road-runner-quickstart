/* Copyright (c) 2023 FIRST. All rights reserved.
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
 *
 *
 * PID controller and IMU codes are copied from
 * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    // 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
    public int startLoc = 1;
    public int spikeMarkLoc = 1; // 1 for left, 2 for center, and 3 for right
    // USE LATER: boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;

    public MecanumDrive drive;

    // camera and sleeve color
    ObjectDetection.PropSide propLocation = ObjectDetection.PropSide.UNKNOWN;

    ObjectDetection propDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;

    // sensing april tag tools
    private AprilTagTest tag = null;
    private int desiredTagNum = -1;

    // road runner variables
    Pose2d startPose;

    /**
     * Set robot starting position: 1 for right and -1 for left.
     */
    public void setRobotLocation() {
        startLoc = 1;
    }

    private void setStartPoses() {
        // road runner variables
        if (1 == startLoc) { // red front
            startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH,
                    3 * Params.HALF_MAT, Math.toRadians(-90.0));
        }

        if (2 == startLoc) { // red back
            startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH,
                    -1 * Params.HALF_MAT, Math.toRadians(-90.0));
        }

        if (3 == startLoc) { //  blue front
            startPose = new Pose2d(6 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH,
                    3 * Params.HALF_MAT, Math.toRadians(90.0));
        }

        if (4 == startLoc) { //  blue back
            startPose = new Pose2d(6 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH,
                    -1 * Params.HALF_MAT, Math.toRadians(90.0));
        }
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setRobotLocation();

        setStartPoses();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        propDetect = new ObjectDetection();

        if (startLoc <= 2) {
            propDetect.setColorFlag(ObjectDetection.ColorS.RED);
        } else {
            propDetect.setColorFlag(ObjectDetection.ColorS.BLUE);
        }

        if (isCameraInstalled) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                    WebcamName.class, webcamName), cameraMonitorViewId);

            camera.setPipeline(propDetect);

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
        drive = new MecanumDrive(hardwareMap, startPose);
        Params.currentPose = startPose; // init storage pose.

        telemetry.addData("left front pos", drive.leftFront.getCurrentPosition());
        telemetry.addData("left back pos", drive.leftBack.getCurrentPosition());
        telemetry.addData("right front pos", drive.rightFront.getCurrentPosition());
        telemetry.addData("right back pos", drive.rightBack.getCurrentPosition());
        telemetry.update();

        //intake = new intakeUnit(hardwareMap, "ArmMotor", "WristServo", "FingerServo");
        //intake.resetArmEncoder();

        sleep(500);

        runtime.reset();
        while ((ObjectDetection.PropSide.UNKNOWN == propLocation) &&
                ((runtime.seconds()) < 3.0)) {
            propLocation = propDetect.getPropPos();
        }
        Logging.log("Parking Lot position: %s", propLocation.toString());

        while (!isStarted()) {
            propLocation = propDetect.getPropPos();
            propLocation = ObjectDetection.PropSide.LEFT; // for test

            switch (propLocation) {
                case LEFT:
                    spikeMarkLoc = 1;
                    desiredTagNum = 1 + (startLoc < 3 ? 3 : 0);
                    break;
                case CENTER:
                case UNKNOWN:
                    spikeMarkLoc = 2;
                    desiredTagNum = 2 + (startLoc < 3 ? 3 : 0);
                    break;
                case RIGHT:
                    spikeMarkLoc = 3;
                    desiredTagNum = 3 + (startLoc < 3 ? 3 : 0);
                    break;
            }

            telemetry.addData("Detected Prop location: ", propLocation);
            telemetry.addData("Desired Tag ID: ", "%d", desiredTagNum);
            telemetry.addData("RR", "imu Heading Yaw = %.1f",
                    drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        tag = new AprilTagTest(drive, hardwareMap, desiredTagNum, webcamName);

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            //intake.armManualMoving(15);
            sleep(150);
            autonomousCore();
            camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
            Logging.log("Autonomous time - total Run Time: " + runtime);
        }
    }

    public void autonomousCore() {
        camera.closeCameraDevice();
        tag.initAprilTag();
        switch (startLoc) {
            case 1:
            case 2:
                autoRedCore();
                break;
            case 3:
            case 4:
                autoBlueCore();
                break;
        }
    }

    private void autoRedCore() {
        // 1. move to central line
        Pose2d pMatCenter = new Pose2d(startPose.position.x - 2 * Params.CHASSIS_HALF_WIDTH + 4 * Params.HALF_MAT, startPose.position.y, startPose.heading.log());
        Vector2d vBackdrop = new Vector2d(-3 * Params.HALF_MAT, -4 * Params.HALF_MAT - Params.BACKDROP_FORWARD);

        Vector2d vAprilTag4 = new Vector2d(vBackdrop.x + Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        Vector2d vAprilTag5 = new Vector2d(vBackdrop.x , vBackdrop.y);
        Vector2d vAprilTag6 = new Vector2d(vBackdrop.x - Params.BACKDROP_SIDEWAYS, vBackdrop.y);

        Logging.log("red backdrop pose y: %2f", vBackdrop.y);
        Logging.log("code number 4 x: %2f", vAprilTag4.x);
        Logging.log("code number 4 y: %2f", vAprilTag4.y);

        Logging.log("robot drive: before strafe pos heading : %.2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: before strafe imu heading : %.2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if (1 == spikeMarkLoc) { // left
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(pMatCenter.position)
                            .turn(Math.PI)
                            .build());
        }
        
        if (2 == spikeMarkLoc) { // center
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(pMatCenter.position)
                            .turn(Math.PI / 2.0)
                            .build());
        }

        if (3 == spikeMarkLoc) // right
        {
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(pMatCenter.position)
                            .build());
        }
        telemetry.addData("left front pos", drive.leftFront.getCurrentPosition());
        telemetry.addData("left back pos", drive.leftBack.getCurrentPosition());
        telemetry.addData("right front pos", drive.rightFront.getCurrentPosition());
        telemetry.addData("right back pos", drive.rightBack.getCurrentPosition());
        telemetry.update();


        Logging.log("robot drive: after strafe pos x : %.2f", drive.pose.position.x);
        Logging.log("robot drive: after strafe pos y : %.2f", drive.pose.position.y);
        
        Logging.log("robot drive: after turn pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // 2. open claw to release purple pixel
        sleep(500);

        //3. close claw
        sleep(100);

        // turn back and facing to backdrop board
        if (1 == spikeMarkLoc) // left
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI)
                            .build());
        }

        if (2 == spikeMarkLoc) { // center
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            /* there should have a bug somewhere in turn()
                               function when using PI/2, it actually turn PI */
                            .turn(-Math.PI / 2 + 0.0001 /* -Math.PI / 2*/)
                            .build());
        }

        Logging.log("robot drive: after turn back pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn back imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // move to the center of second mat.
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(vBackdrop.x, drive.pose.position.y))
                        .build()
        );

        // correct heading
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turn(-drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))
                        .build());
        Logging.log("robot drive: after turn correction pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn correction imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // move forward to backdrop board
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(vAprilTag5.y)
                        //.lineToY(vAprilTag5.y)
                        .build()
        );
        Logging.log("robot drive: arrive backdrop pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: arrive backdrop imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // shift to AprilTag
        if (1 == spikeMarkLoc) // left, mark number 4
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vAprilTag4)
                            .build()
            );
        }

        if (3 == spikeMarkLoc) // right, mark number 6
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vAprilTag6)
                            .build()
            );
        }

        tag.autoDriveToAprilTag();
        // drop pixel
    }


    // 3 = startLoc, or 4
    private void autoBlueCore(){
        Pose2d pMatCenter = new Pose2d(startPose.position.x + 2 * Params.CHASSIS_HALF_WIDTH - 4 * Params.HALF_MAT, startPose.position.y, startPose.heading.log());
        Vector2d vBackdrop = new Vector2d(3 * Params.HALF_MAT, -4 * Params.HALF_MAT - Params.BACKDROP_FORWARD);

        Vector2d vAprilTag1 = new Vector2d(vBackdrop.x + Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        Vector2d vAprilTag2 = new Vector2d(vBackdrop.x , vBackdrop.y);
        Vector2d vAprilTag3 = new Vector2d(vBackdrop.x - Params.BACKDROP_SIDEWAYS, vBackdrop.y);

        Logging.log("Blue backdrop pose y: %2f", vBackdrop.y);
        Logging.log("code number 4 x: %2f", vAprilTag1.x);
        Logging.log("code number 4 y: %2f", vAprilTag1.y);

        Logging.log("robot drive: before strafe pos heading : %.2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: before strafe imu heading : %.2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if (1 == spikeMarkLoc) { // left
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(pMatCenter.position)
                            .turn(Math.PI)
                            .build());
        }

        if (2 == spikeMarkLoc) { // center
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(pMatCenter.position)
                            .turn(Math.PI / 2.0)
                            .build());
        }

        if (3 == spikeMarkLoc) // right
        {
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(pMatCenter.position)
                            .build());
        }
        telemetry.addData("left front pos", drive.leftFront.getCurrentPosition());
        telemetry.addData("left back pos", drive.leftBack.getCurrentPosition());
        telemetry.addData("right front pos", drive.rightFront.getCurrentPosition());
        telemetry.addData("right back pos", drive.rightBack.getCurrentPosition());
        telemetry.update();


        Logging.log("robot drive: after strafe pos x : %.2f", drive.pose.position.x);
        Logging.log("robot drive: after strafe pos y : %.2f", drive.pose.position.y);

        Logging.log("robot drive: after turn pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // 2. open claw to release purple pixel
        sleep(500);

        //3. close claw
        sleep(100);

        // turn back and facing to backdrop board
        if (2 == spikeMarkLoc) { // center
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            /* there exists a bug somewhere in turn()
                               function that makes it so when turning PI/2, it actually turn PI */
                            .turn(Math.PI / 2 -  0.0001 /* -Math.PI / 2*/)
                            .build());
        }
        if (3 == spikeMarkLoc) { // right
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI)
                            .build()
            );
        }

        Logging.log("robot drive: after turn back pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn back imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // move to the center of second mat.
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(vBackdrop.x, drive.pose.position.y))
                        .build()
        );

        // correct heading straight towards backdrop
        double imuAngle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Logging.log("correction angle: %2f", Math.copySign(Math.PI, imuAngle) - imuAngle);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turn(Math.copySign(Math.PI, imuAngle) -imuAngle)
                        .build());

        Logging.log("robot drive: after turn correction pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn correction imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // move forward to backdrop board
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(vAprilTag2.y)
                        .build()
        );
        Logging.log("robot drive: arrive backdrop pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: arrive backdrop imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        sleep(200);

        // shift to AprilTag
        /*if (1 == spikeMarkLoc) // left, tag number 1
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vAprilTag1)
                            .build()
            );
        }

        if (3 == spikeMarkLoc) // right, tag number 3
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vAprilTag3)
                            .build()
            );
        }

         */

/*
        double startTime = runtime.milliseconds();
        while(!tag.targetFound && (runtime.milliseconds() - startTime < 100)){
            tag.autoDriveToAprilTag();
        }
        Logging.log("April 1st Tag found? %s ", tag.targetFound? "Yes" : "No");


        while(tag.targetFound){
            tag.autoDriveToAprilTag();
            while(!tag.targetFound && (runtime.milliseconds() - startTime < 100)){
                tag.autoDriveToAprilTag();
            }
            startTime = runtime.milliseconds();
            Logging.log("April Tag found? %s ", tag.targetFound? "Yes" : "No");
        }

 */
        tag.autoDriveToAprilTag();

        // drop pixel
    }
}
