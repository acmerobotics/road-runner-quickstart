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
 *      Servo motors:
 *          "FingerServo"
 *          "WristServo"
 *          "SwitchServo"
 *
 *      One cameras:
 *          "Webcam 1"
 */

@Autonomous(name="Auto Red Front", group="Concept")
//@Disabled
public class AutoRedFront extends LinearOpMode {

    /** 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public int startLoc = 1;
    /** blue: 1; red: -1
     */
    private int blueOrRed = 1; // blue: 1; red: -1
    /** front: 1; back -1
     */
    private int frontOrBack = 1; // front: 1; back -1
    /** 1 for left, 2 for center, and 3 for right
     */
    public int spikeMarkLoc = 1; // 1 for left, 2 for center, and 3 for right
    // USE LATER: boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;

    private MecanumDrive drive;

    // camera and sleeve color
    ObjectDetection.PropSide propLocation = ObjectDetection.PropSide.UNKNOWN;

    ObjectDetection propDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;

    // sensing april tag tools
    private AprilTagTest tag = null;
    /**
     * blue: 1,2,3; red: 4,5,6
     */
    private int desiredTagNum = -1; // blue: 1,2,3; red: 4,5,6

    // road runner variables
    Pose2d startPose;

    final int WAIT_ALLIANCE_SECONDS = 1;

    /**
     * Set robot starting location on the field:
     * 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public void setRobotLocation() {
        startLoc = 1;
    }

    /**
     * Set robot starting position variables according to start locations:
     * @param startLocation : the value of robot location in the field.
     *                      1 for Red Front, 2 for Red back,
     *                      3 for Blue Front, and 4 for Blue back
     */
    private void setStartPoses(int startLocation) {
        // road runner variables
        switch(startLocation) {
            case 1: // red front
                blueOrRed = -1;
                frontOrBack = 1;
                break;

            case 2: // red back
                blueOrRed = -1;
                frontOrBack = -1;
                break;

            case 3: //  blue front
                blueOrRed = 1;
                frontOrBack = 1;
                break;

            case 4: //  blue back
                blueOrRed = 1;
                frontOrBack = -1;
                break;
        }
        startPose = new Pose2d((6 * Params.HALF_MAT - Params.CHASSIS_LENGTH / 2) * blueOrRed,
                //Params.HALF_MAT + (3 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH) * frontOrBack,
                Params.HALF_MAT + 2 * Params.HALF_MAT * frontOrBack,
                Math.toRadians(90.0 + 90.0 * blueOrRed));
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setRobotLocation();

        setStartPoses(startLoc);

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

        intake = new intakeUnit(hardwareMap, "ArmMotor", "WristServo",
                "FingerServo", "SwitchServo");
        intake.resetArmEncoder();

        runtime.reset();
        while ((ObjectDetection.PropSide.UNKNOWN == propLocation) &&
                ((runtime.seconds()) < 3.0)) {
            propLocation = propDetect.getPropPos();
        }

        while (!isStarted()) {
            propLocation = propDetect.getPropPos();
            //propLocation = ObjectDetection.PropSide.CENTER; // TODO: remove after temp test

            switch (propLocation) {
                case LEFT:
                    spikeMarkLoc = 1;
                    break;
                case CENTER:
                case UNKNOWN:
                    spikeMarkLoc = 2;
                    break;
                case RIGHT:
                    spikeMarkLoc = 3;
                    break;
            }
            desiredTagNum = spikeMarkLoc + (3 - blueOrRed * 3) / 2; // blue: 1,2,3; red: 4,5,6

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
            intake.autonomousInit();
            camera.closeCameraDevice(); // close camera for spike mark location checking

            autonomousCore();

            intake.parkingPositions(); // Motors are at intake positions at the beginning of Tele-op
            intake.fingerStop();
            sleep(1000);

            camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
            Logging.log("Autonomous time - total Run Time: " + runtime);
        }
    }

    private void autonomousCore() {
        autoCore();
    }

    private void autoCore() {
        int checkStatus = desiredTagNum * frontOrBack;

        if ((6 == checkStatus) || (-4 == checkStatus) || (1 == checkStatus) ||(-3 == checkStatus))
        {
            return;
        }
        double pausePoseY = -2 * Params.HALF_MAT - 6;
        Vector2d vMatCenter = new Vector2d(blueOrRed * 3 * Params.HALF_MAT, startPose.position.y);
        Vector2d vParkPos = new Vector2d(blueOrRed * ((3 - 2 * frontOrBack) * Params.HALF_MAT + 2), -3.5 * Params.HALF_MAT);
        Vector2d vBackdrop = new Vector2d(blueOrRed * 3 * Params.HALF_MAT, -4 * Params.HALF_MAT);

        Vector2d vAprilTag = null;
        double rightBucketShift = 2.0; // yellow pixel is in the right bucket.
        if (blueOrRed > 0) {
            vAprilTag = new Vector2d(vBackdrop.x + (desiredTagNum - 2) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        }
        else {
            vAprilTag = new Vector2d(vBackdrop.x + (5 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        }
        Vector2d vCheckingAprilTagPose = new Vector2d(vAprilTag.x, vAprilTag.y + 10);
        Vector2d vDropYellow = new Vector2d(vAprilTag.x + rightBucketShift, vAprilTag.y);

        Vector2d vDropPurple = null;
        double xDelta = -3.0;
        double yDelta = 10.0;

        switch (checkStatus) {
            case 1:
                if (frontOrBack > 0) {

                }
                break;
            case -1:
                    xDelta = 9.0;
                    yDelta = 10.0;
                break;

            case 5:
            case -5:
            case 2:
            case -2:
                xDelta = 5.5;
                yDelta = (frontOrBack > 0)? 5.0 : 0;
                break;
            case 3:
            case -3:
                break;

            case 4:
                    xDelta = 8.0;
                    yDelta = 8.5;
                break;
            case -4:
            case 6:
                    xDelta = -3;
                    yDelta = -10.0;
                break;
            case -6:
                    xDelta = 8.0;
                    yDelta = 15.0;
                break;
        }
        vDropPurple = new Vector2d(blueOrRed * (3 * Params.HALF_MAT + xDelta), startPose.position.y + frontOrBack * yDelta);


        Logging.log("red backdrop pose y: %2f", vBackdrop.y);
        Logging.log("code number 4 x: %2f", vAprilTag.x);
        Logging.log("code number 4 y: %2f", vAprilTag.y);
        Logging.log(" desired tag = %d, frontOrBack = %d, xDelta = %.2f, yDelta = %.2f ", desiredTagNum, frontOrBack, xDelta, yDelta);

        Logging.log("robot drive: before strafe pos heading : %.2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: before strafe imu heading : %.2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        Logging.log("robot drive: start position: x=%.2f, y=%.2f", drive.pose.position.x, drive.pose.position.y);

        // move forward
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToXConstantHeading(startPose.position.x - blueOrRed * 6) // drive forward 6 inch to leave wall
                        .build()
        );

        intake.pushPropPose();
        sleep(2000);

        if ((4 == checkStatus) || (5 == checkStatus) ||
                (-5 == checkStatus) || (-6 == checkStatus) ||
                (2 == checkStatus) || (3 == checkStatus) ||
                (-2 == checkStatus) || (-1 == checkStatus)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vDropPurple)
                            .build()
            );
        }

        Logging.log("robot drive: after drop off purple x position: %.2f", drive.pose.position.x);
        Logging.log("robot drive: after drop off purple required x position: %.2f", vDropPurple.x);
        Logging.log("robot drive: after drop off purple y position: %.2f", drive.pose.position.y);
        Logging.log("robot drive: after drop off purple required Y position: %.2f", vDropPurple.y);

        Logging.log("robot drive: after turn pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // drop off the purple pixel by arm and wrist actions
        dropPurpleAction();

        if ((2 == checkStatus) || (5 == checkStatus)){
            intake.setArmModeRunToPosition(intake.ARM_POS_READY_FOR_HANG);
        }
        else {
            intake.underTheBeam();
        }
        sleep(1000);

        // turn back and facing to backdrop board
        if ((2 == spikeMarkLoc) || (1 == spikeMarkLoc)) { // center
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // there is a bug somewhere in turn()
                            //   function when using PI/2, it actually turn PI */
                            .turn((Math.PI / 2) * blueOrRed + 0.00001)
                            .build());
        }

        intake.underTheBeam();
        sleep(500);

        Logging.log("robot drive: after turn back pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn back imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //drive.updatePoseEstimate();
        Logging.log("robot drive: after turn head to backdrop x position: %2f", drive.pose.position.x);

        // move to the center of second mat to go through gate.
        if (frontOrBack > 0) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vMatCenter)
                            .build()
            );
        }

        sleep(1000);

        // fine tune heading angle
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turn(-drive.pose.heading.log() - Math.PI / 2)
                        .build());
        Logging.log("robot drive: after strafe to mat center x position: %2f", drive.pose.position.x);

        Logging.log("robot drive: after turn correction pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn correction imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if (frontOrBack > 0) {
            sleep(WAIT_ALLIANCE_SECONDS * 100);
        }
        tag.initAprilTag();

        // move forward to backdrop board
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(pausePoseY)
                        .strafeTo(vCheckingAprilTagPose)
                        .build()
        );

        Logging.log("robot drive: check april tag x position: %2f", drive.pose.position.x);
        Logging.log("robot drive: check april tag y position: %2f", drive.pose.position.y);
        Logging.log("robot drive: check april tag required y position: %2f", vCheckingAprilTagPose.y);

        Logging.log("robot drive: arrive backdrop pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: arrive backdrop imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        intake.readyToDropYellow();

        // if can not move based on April tag, moved by road runner.
        //if (!tag.autoDriveToAprilTag()) {
        if (true) {
            // shift to AprilTag
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(vDropYellow)
                                .build()
                );
        }
        Logging.log("robot drive: arrive backdrop x position: %2f", drive.pose.position.x);
        Logging.log("robot drive: arrive backdrop vAprilTag x position: %2f", vAprilTag.x);

        Logging.log("robot drive: arrive backdrop y position: %2f", drive.pose.position.y);
        Logging.log("robot drive: arrive backdrop vAprilTag y position: %2f", vAprilTag.y);
        Logging.log("robot drive: arrive backdrop required position: x=%2f, y=%2f", vDropYellow.x, vDropYellow.y);


        // drop pixel
        dropYellowAction();

        intake.setArmModeRunToPosition(intake.ARM_POS_DROP); // lift arm to ensure yellow pixel is dropped.
        sleep(500);

        Logging.log("robot drive: drop yellow x position: %2f", drive.pose.position.x);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(vParkPos.y) //move back a little bit to left backdrop board.
                        .strafeTo(vParkPos)
                        .build()
        );

        Logging.log("robot drive: parking x position: %2f", drive.pose.position.x);
    }

    private void dropPurpleAction() {
        // 1. arm and wrist at correct positions
        intake.readyToDropPurple();
        sleep(2000);

        // 2. open switch
        intake.setSwitchPosition(intake.SWITCH_RELEASE_PURPLE);
        sleep(1500);
    }
    private void dropYellowAction(){
        intake.readyToDropYellow();
        sleep(500);
        intake.setSwitchPosition(intake.SWITCH_RELEASE_YELLOW);
        sleep(500);
        intake.setSwitchPosition(intake.getArmPosition() - 500);
        sleep(500);
    }
}
