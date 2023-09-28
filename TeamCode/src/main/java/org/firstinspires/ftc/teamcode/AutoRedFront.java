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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    public int startLoc = 2;
    public int spikeMarkLoc = 1; // 1 for left, 2 for center, and 3 for right
    boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ArmClawUnit armClaw = new ArmClawUnit();

    public MecanumDrive drive;

    // camera and sleeve color
    ObjectDetection.PropSide propLocation = ObjectDetection.PropSide.UNKNOWN;

    ObjectDetection propDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;

    // road runner variables
    Pose2d startPose;

    // pre cone to medium junction
    Pose2d poseRedBackDropCenter;

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

        // camera for sleeve color detect, start camera at the beginning.
        webcamName = "Webcam 1";

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

        armClaw.init(hardwareMap, "ArmMotor", "ClawServo");
        armClaw.resetArmEncoder();

        sleep(500);
        armClaw.clawClose();
        sleep(500);
        armClaw.armManualMoving(15);
        sleep(500);

        runtime.reset();
        while ((ObjectDetection.PropSide.UNKNOWN == propLocation) &&
                ((runtime.seconds()) < 3.0)) {
            propLocation = propDetect.getPropPos();
        }
        Logging.log("Parking Lot position: %s", propLocation.toString());

        while (!isStarted()) {
            propLocation = propDetect.getPropPos();
            telemetry.addData("Detected Prop location: ", propLocation);
            telemetry.addData("RR", "imu Heading = %.1f",
                    Math.toDegrees(drive.pose.heading.log()));
            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

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

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            armClaw.armManualMoving(15);
            sleep(150);
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
                //autoRedBackCore();
                autoRedFrontCore();
                break;
            case 3:
                autoBlueFrontCore();
                break;
            case 4:
                //autoBlueBackCore();
                autoBlueFrontCore();
                break;
        }
    }


    private void autoRedFrontCore() {

        // 1. move to central line
        //Pose2d poseMatCenter = new Pose2d(-3 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH, 3 * Params.HALF_MAT, startPose.heading.log());
        Vector2d poseMatCenter = new Vector2d(-3 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH, 3 * Params.HALF_MAT);
        Vector2d poseBackdropRed = new Vector2d(-3 * Params.HALF_MAT, poseMatCenter.y - Params.BACK_DISTANCE);
        Vector2d poseQRCode4 = new Vector2d(-35, 6);
        Vector2d poseQRCode5 = new Vector2d(-3 * Params.HALF_MAT , -6 * Params.HALF_MAT - Params.BACKDROP_FORWARD);
        Vector2d poseQRCode6 = new Vector2d(-3 * Params.HALF_MAT + Params.BACKDROP_SIDEWAYS, (-4 * Params.HALF_MAT - Params.BACKDROP_FORWARD) / 12);
        if (2 == startLoc) {
            poseMatCenter = new Vector2d(-3 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH, -1 * Params.HALF_MAT);
        }
        poseRedBackDropCenter = new Pose2d(-3 * Params.HALF_MAT, -4 * Params.HALF_MAT, Math.toRadians(-90.0));
        Logging.log("red backdrop pose y: %2f", poseBackdropRed.y);
        Logging.log("code number 4 x: %2f", poseQRCode4.x);
        Logging.log("code number 4 y: %2f", poseQRCode4.y);


        if (1 == spikeMarkLoc) {// left
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(poseMatCenter)
                            .turn(Math.PI)
                            //.lineToYConstantHeading(poseMatCenter.y + Params.SPIKE_GAP)
                            .build());

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);

            //4. back to center
            /*Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI)
                            .lineToYConstantHeading(poseMatCenter.y)
                            .build());

             */


             /*
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI)
                            .lineToYConstantHeading(poseRedBackDropCenter.position.y)
                            .build());


            // 4. turn 180
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(180.0))
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            // 4. move to Backdrop
            trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(8 * Params.HALF_MAT)
                    .addDisplacementMarker(4 * Params.HALF_MAT, armClaw::armLift)
                    .strafeLeft(Params.HALF_MAT / 2.0)
                    .build();
            drive.followTrajectorySequence(trajSeq1);

            armClaw.clawOpen();
            */
        }


        if (2 == spikeMarkLoc) {
            // center
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(poseMatCenter)
                            .turn(Math.PI / 2.0)
                            .build());

            // 2. open claw, to release the purple pixel
            armClaw.clawOpen();
            sleep(100);

            // 3. close claw to pick-up the yellow pixel
            armClaw.clawClose();
            sleep(100);

            // 4. back to center
            /*Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToXConstantHeading(poseMatCenter.x)
                            .turn(- Math.PI / 2.0)
                            .build());

             */


            /*
            // 0. drive to center
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseCenterLine)
                    .build();
            drive.followTrajectory(traj1);

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
            */
        }


        if (3 == spikeMarkLoc) // right
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(poseMatCenter)
                            .lineToYConstantHeading(poseMatCenter.y - Params.SPIKE_GAP)
                            .build());

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(500);

            //3. close claw
            armClaw.clawClose();
            sleep(100);

            //4. back to center position
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(poseMatCenter.y)
                            .build());


        }
        //go to backdrop location
        sleep(250);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(poseBackdropRed.y / 12)
                        .build()
        );

        sleep(4000);
        //go to drop position
        if (1 == spikeMarkLoc) //mark number 4
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToY(-67)
                            //.strafeTo(poseQRCode4)
                            .build()
            );
        }
        if (2 == spikeMarkLoc) //mark number 5
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(poseQRCode5.y)
                            .build()
            );
        }
        if (3 == spikeMarkLoc) //mark number 6
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(-poseQRCode6.y)
                            .strafeTo(poseQRCode6)
                            .build()
            );
        }
        armClaw.clawOpen();


    }


    // 3 = startLoc, or 4
    private void autoBlueFrontCore(){

        // 1. move to central line
        Pose2d poseMatCenter = new Pose2d(3 * Params.HALF_MAT, 3 * Params.HALF_MAT, startPose.heading.log());

        if (4 == startLoc) {
            poseMatCenter = new Pose2d(3 * Params.HALF_MAT, -1 * Params.HALF_MAT, startPose.heading.log());
        }

        if (3 == spikeMarkLoc) {// right
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToXConstantHeading(poseMatCenter.position.x)
                            .lineToYConstantHeading(poseMatCenter.position.y + Params.SPIKE_GAP)
                            .build());

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);
        }

        if (2 == spikeMarkLoc) { // center
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToXConstantHeading(poseMatCenter.position.x - Params.SPIKE_GAP)
                            .turn(Math.toRadians(90.0))
                            .build());

            // 2. open claw, to release the purple pixel
            armClaw.clawOpen();
            sleep(100);

            // 3. close claw to pick-up the yellow pixel
            armClaw.clawClose();
            sleep(100);

        }

        if (1 == spikeMarkLoc) // left
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToXConstantHeading(poseMatCenter.position.x)
                            .turn(Math.toRadians(180.0))
                            .lineToYConstantHeading(poseMatCenter.position.y - Params.SPIKE_GAP)
                            .build());

            // 2. open claw to release purple pixel
            armClaw.clawOpen();
            sleep(100);

            //3. close claw
            armClaw.clawClose();
            sleep(100);


        }
    }
}
