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

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    /**
     * blue: 1,2,3; red: 4,5,6
     */
    private int desiredTagNum = 0; // blue: 1,2,3; red: 4,5,6
    private int checkStatus = 0;
    final private double BUCKET_SHIFT = 2.0; // yellow pixel is in the right bucket.
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

    // road runner variables
    Pose2d startPose;

    final int WAIT_ALLIANCE_SECONDS = 3;

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
        Params.blueOrRed = blueOrRed;

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
            //propLocation = ObjectDetection.PropSide.LEFT; // TODO: remove after temp test

            int spikeMarkLoc = 1; // 1 for left, 2 for center, and 3 for right
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
            checkStatus = desiredTagNum * frontOrBack;

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

            Params.currentPose = drive.pose; // storage end pose of autonomous
            intake.parkingPositions(); // Motors are at intake positions at the beginning of Tele-op
            intake.fingerStop();
            sleep(1000);

            camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
            Logging.log("Autonomous time - total Run Time: " + runtime);
        }
    }

    private void autonomousCore() {
        autoCore();
        intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void autoCore() {
        Vector2d startArmFlip = new Vector2d(startPose.position.x - blueOrRed * 6, startPose.position.y);

        double pausePoseY = -2 * Params.HALF_MAT - 6;
        Vector2d vMatCenter = new Vector2d(blueOrRed * 3 * Params.HALF_MAT, startPose.position.y);
        Vector2d vParkPos = new Vector2d(blueOrRed * ((3 - 2 * frontOrBack) * Params.HALF_MAT - frontOrBack * ((frontOrBack > 0)? 0 : 3)), -3.5 * Params.HALF_MAT);
        Vector2d vBackdrop = new Vector2d(blueOrRed * 3 * Params.HALF_MAT, -4 * Params.HALF_MAT);

        Vector2d vAprilTag = null;

        if (blueOrRed > 0) {
            vAprilTag = new Vector2d(vBackdrop.x + (2 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        }
        else {
            vAprilTag = new Vector2d(vBackdrop.x + (5 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        }
        Vector2d vCheckingAprilTagPose = new Vector2d(vAprilTag.x, vAprilTag.y + 10);
        Vector2d vDropYellow = new Vector2d(vAprilTag.x + BUCKET_SHIFT, vAprilTag.y);

        Vector2d vDropPurple = null;
        double xDelta = -3.0;
        double yDelta = 10.0;

        switch (checkStatus) {
            case 5:
            case -5:
            case 2:
            case -2:
                // pass the test
                xDelta = 5.0;
                yDelta = (frontOrBack > 0) ? 5.0 : 0;
                break;
            case -1:
            case 4:
                // pass the test
                xDelta = 8.0;
                yDelta = 10.0;
                break;
            case -3:
            case 1:
            case -4:
            case 6:
                xDelta = -4; // 0;
                yDelta = 3;//5;
                startArmFlip = new Vector2d(blueOrRed * (3 * Params.HALF_MAT + xDelta), startPose.position.y + frontOrBack * 10);
                break;
            case 3:
            case -6:
                // pass the test
                xDelta = 8.0;
                yDelta = 15.0;
                break;
        }
        vDropPurple = new Vector2d(blueOrRed * (3 * Params.HALF_MAT + xDelta), startPose.position.y + frontOrBack * yDelta);

        Logging.log("check status = %d, xDelta = %.2f, yDelta = %.2f ", checkStatus, xDelta, yDelta);
        logVector("Back drop pose", vBackdrop);
        logVector("April tag", vAprilTag);

        logRobotHeading("robot drive: before strafe");
        logVector("robot drive: start position", startPose.position);

        // move forward
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeTo(startArmFlip) // drive forward several inch to leave wall
                        .build()
        );

        logVector("robot drive: arm to push pose", drive.pose.position);
        logVector("robot drive: start Arm Flip pose required", startArmFlip);

        // Near gate cases
        if((6 == checkStatus) || (-3 == checkStatus) || (1 == checkStatus) || (-4 == checkStatus)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI / 2 * frontOrBack * blueOrRed)
                            .build()
            );
            logRobotHeading("robot drive: after turn before arm flip");
            logVector("robot drive: after turn pose starting Arm Flip required", drive.pose.position);
            logVector("robot drive: after turn pose starting Arm Flip required", startArmFlip);
        }

        intake.pushPropPose();
        sleep(2000);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vDropPurple)
                        .build()
        );

        logVector("robot drive: drop purple pose", drive.pose.position);
        logVector("robot drive: drop purple pose required", vDropPurple);

        // drop off the purple pixel by arm and wrist actions
        dropPurpleAction();

        if ((2 == checkStatus) || (5 == checkStatus)) {
            intake.setArmModeRunToPosition(intake.ARM_POS_READY_FOR_HANG);
            sleep(1000);
        } else {
            intake.underTheBeam();
            sleep(300);
        }

        // there is a bug somewhere in turn() function when using PI/2, it actually turn PI */
        double turnAngleToDrop = 0;
        if((-4 == checkStatus) || (-3 == checkStatus)) {
            turnAngleToDrop = -blueOrRed * (Math.PI + 0.00001);

            // move back a little bit before turn to avoid hitting gate
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(drive.pose.position.y - 5)
                            .build());
            logVector("robot drive: back after drop purple", drive.pose.position);
            logVector("robot drive: back after drop purple required", vDropPurple);
        } else {
            turnAngleToDrop = (Math.PI / 2) * blueOrRed + 0.00001;
        }

        if((6 != checkStatus) && (1 != checkStatus)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(turnAngleToDrop)
                            .build());
            logRobotHeading("robot drive: turn after drop purple");
            logVector("robot drive: turn after drop purple", drive.pose.position);
            logVector("robot drive: turn after drop purple required", vDropPurple);
        }

        // flip down the arm to get ready to go through the gate
        if ((2 == checkStatus) || (5 == checkStatus)) {
            intake.underTheBeam();
            sleep(500);
        }

        // move to the center of second mat to go through gate.
        if (frontOrBack > 0) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vMatCenter)
                            .build()
            );

            logVector("robot drive: drive.pose move to 2nd mat center", drive.pose.position);
            logVector("robot drive: move to 2nd mat center required", vMatCenter);
        }

        // fine tune heading angle
        Actions.runBlocking(
                new ParallelAction(
                        // Paral 1. turn on camera for april tag detect
                        new TurnOnCamera(),

                        // Paral 2.
                        new SequentialAction(
                                // Seq a. fine tune heading angle before long travel
                                drive.actionBuilder(drive.pose)
                                        .turn(-drive.pose.heading.log() - Math.PI / 2)
                                        .build(),

                                // Seq b. waiting alliance move out the way if at front side
                                new SleepAction((frontOrBack > 0)? WAIT_ALLIANCE_SECONDS : 0),

                                // Seq c. strafe to april tag pose to check april tag
                                drive.actionBuilder(drive.pose)
                                        .lineToYConstantHeading(pausePoseY)
                                        .strafeTo(vCheckingAprilTagPose)
                                        .build()
                        )
                )
        );
        logRobotHeading("robot drive: fine turn for heading correction");
        logVector("robot drive: drive.pose check april tag", drive.pose.position);
        logVector("robot drive: check april tag required", vCheckingAprilTagPose);
        logRobotHeading("robot drive: check april tag");

        if(intake.getArmPosition() > intake.ARM_POS_CAMERA_READ) {
            intake.setArmCountPosition(intake.ARM_POS_CAMERA_READ); // lift arm to avoid blocking camera
            sleep(500);
        }
        Pose2d aprilTagPose = tag.updatePoseAprilTag(desiredTagNum);
        logVector("robot drive: april tag location from camera", aprilTagPose.position);
        logVector("robot drive: drop yellow pose required before adjust", vDropYellow);

        // if can not move based on April tag, moved by road runner.
        if (tag.targetFound) {
            // adjust yellow drop-off position according to april tag location info from camera
            vDropYellow = new Vector2d(drive.pose.position.x - aprilTagPose.position.x + BUCKET_SHIFT,
                    drive.pose.position.y - aprilTagPose.position.y + Params.AUTO_DISTANCE_TO_TAG);
            logVector("robot drive: drop yellow pose required after april tag adjust", vDropYellow);
        }

        intake.readyToDropYellow();

        // shift to AprilTag
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vDropYellow)
                        .build()
        );
        logVector("robot drive: drive.pose drop yellow", drive.pose.position);
        logVector("robot drive: check drop yellow required", vDropYellow);

        logVector("robot drive: april tag required", vAprilTag);

        // drop pixel
        dropYellowAction();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(vParkPos.y) //move back a little bit to left backdrop board.
                        .strafeTo(vParkPos)
                        .build()
        );
        logVector("robot drive: drive.pose parking", drive.pose.position);
        logVector("robot drive: parking required", vParkPos);
    }

    private void dropPurpleAction() {
        // 1. arm and wrist at correct positions
        intake.readyToDropPurple();
        sleep(500);

        // 2. open switch
        intake.setSwitchPosition(intake.SWITCH_RELEASE_PURPLE);
        sleep(500);
    }
    private void dropYellowAction(){
        intake.readyToDropYellow();
        sleep(100);
        intake.setSwitchPosition(intake.SWITCH_RELEASE_YELLOW);
        sleep(500);
        intake.setArmCountPosition(intake.getArmPosition() - 500);
        sleep(500);
    }

    private void logVector(String sTag, Vector2d vXY) {
        String vectorName = vXY.toString();
        Logging.log("%s: %s", sTag, vectorName);
    }

    private void logRobotHeading(String sTag) {
        Logging.log("%s drive.pose: %.2f", sTag, Math.toDegrees(drive.pose.heading.log()));
        Logging.log("%s imu: %.2f", sTag, drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - Math.toDegrees(startPose.heading.log()));
    }

    public class TurnOnCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            tag.initAprilTag();
            return false;
        }
    }
}
