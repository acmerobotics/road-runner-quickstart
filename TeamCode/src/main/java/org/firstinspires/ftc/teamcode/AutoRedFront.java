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

        intake = new intakeUnit(hardwareMap, "ArmMotor", "WristServo",
                "FingerServo", "SwitchServo");

        intake.resetArmEncoder();

        sleep(500);
        intake.autonomousInit();

        runtime.reset();
        while ((ObjectDetection.PropSide.UNKNOWN == propLocation) &&
                ((runtime.seconds()) < 3.0)) {
            propLocation = propDetect.getPropPos();
        }
        Logging.log("Parking Lot position: %s", propLocation.toString());

        while (!isStarted()) {
            propLocation = propDetect.getPropPos();
            propLocation = ObjectDetection.PropSide.CENTER; // TODO: remove after temp test

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
            camera.closeCameraDevice(); // close camera for spike mark location checking

            //sleep(150);

            autonomousCore();
            intake.parkingPositions(); // Motors are at intake positions at the beginning of Tele-op
            sleep(2000);

            camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
            Logging.log("Autonomous time - total Run Time: " + runtime);
        }
    }

    private void autonomousCore() {
        tag.initAprilTag();
        autoCore();
    }

    private void autoCore() {
        // 1. move to central line
        double centerPoseX = blueOrRed * (3.5 * Params.HALF_MAT - 1); // x destination for center spike
        Pose2d pMatCenter = new Pose2d(3 * blueOrRed * Params.HALF_MAT, startPose.position.y, startPose.heading.log());
        Vector2d vParkPos = new Vector2d(blueOrRed * (0.5 * Params.HALF_MAT - 3), -3.5 * Params.HALF_MAT);
        Vector2d vBackdrop = new Vector2d(3 * blueOrRed * Params.HALF_MAT, -4 * Params.HALF_MAT - Params.BACKDROP_FORWARD);

        Vector2d vAprilTag4 = new Vector2d(vBackdrop.x + Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        Vector2d vAprilTag5 = new Vector2d(vBackdrop.x , vBackdrop.y);
        Vector2d vAprilTag6 = new Vector2d(vBackdrop.x - Params.BACKDROP_SIDEWAYS, vBackdrop.y);

        Logging.log("red backdrop pose y: %2f", vBackdrop.y);
        Logging.log("code number 4 x: %2f", vAprilTag4.x);
        Logging.log("code number 4 y: %2f", vAprilTag4.y);

        Logging.log("robot drive: before strafe pos heading : %.2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: before strafe imu heading : %.2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // move forward
        if (1 == spikeMarkLoc) { // left
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI / 2)
                            .build());
        }

        if (2 == spikeMarkLoc) { // center
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .lineToXConstantHeading(centerPoseX)
                            .build()
            );
        }

        if (3 == spikeMarkLoc) // right
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(- Math.PI / 2.0)
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

        // drop off the purple pixel by arm and wrist actions
        dropPurpleAction();
        sleep(1000);
        intake.underTheBeam();

        // turn back and facing to backdrop board
        if (1 == spikeMarkLoc && blueOrRed > 0) // left red
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI)
                            .build());
        }

        if (2 == spikeMarkLoc) { // center
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // there is a bug somewhere in turn()
                            //   function when using PI/2, it actually turn PI */
                            .turn((Math.PI / 2) * blueOrRed + 0.00001)
                            .build());
        }

        if (3 == spikeMarkLoc && blueOrRed < 0){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI)
                            .build()
            );
        }
        Logging.log("robot drive: after turn back pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn back imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //drive.updatePoseEstimate();
        Logging.log("robot drive: after turn back x position: %2f", drive.pose.position.x);

        // move to the center of second mat.
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(pMatCenter.position.x, drive.pose.position.y))
                        .build()
        );

        // fine tune heading angle
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turn(-drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-Math.PI / 2)
                        .build());
        Logging.log("robot drive: after turn correction pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: after turn correction imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // move forward to backdrop board
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(vAprilTag5.y + 9)
                        //.lineToY(vAprilTag5.y)
                        .build()
        );
        Logging.log("robot drive: arrive backdrop pos heading : %2f", Math.toDegrees(drive.pose.heading.log()));
        Logging.log("robot drive: arrive backdrop imu heading : %2f", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        intake.readyToDropYellow();
        // shift to AprilTag
        if (1 == spikeMarkLoc) // left, mark number 4
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(vAprilTag4)
                            .build()
            );
        }

        if(2 == spikeMarkLoc) //center, mark number 5
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(vAprilTag5.y)
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
        dropYellowAction();
        Logging.log("drive pose x:%2f", drive.pose.position.x);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(vParkPos.y)
                        .strafeTo(vParkPos)
                        .build()
        );



    }

    private void autoRedCore() {
        // 1. move to central line
        double centerPoseX = ((startLoc>2)? -1 : 1) * 4 * Params.HALF_MAT;
        spikeMarkLoc = 2;
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

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToXConstantHeading(pMatCenter.position.x)
                        .build()
        );

        if (1 == spikeMarkLoc) { // left
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(Math.PI / 2)
                            .build());
        }
        
        if (2 == spikeMarkLoc) { // center
            //left empty because already executed
        }

        if (3 == spikeMarkLoc) // right
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(- Math.PI / 2.0)
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

        // drop off the purple pixel by arm and wrist actions
        dropPurpleAction();

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

        if(2 == spikeMarkLoc) //center, mark number 5
        {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(vAprilTag5.y)
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

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineToXConstantHeading(pMatCenter.position.x)
                        .build()
        );

        if (1 == spikeMarkLoc) { // left
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .turn(Math.PI / 2)
                            .build());
        }

        if (2 == spikeMarkLoc) { // center
            // left empty because already executed
        }

        if (3 == spikeMarkLoc) // right
        {
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .turn(- Math.PI / 2)
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

    private void dropPurpleAction() {
        // 1. arm and wrist at correct positions
        intake.readyToDropPurple();
        sleep(2500);

        // 2. open switch
        intake.setSwitchPosition(intake.SWITCH_RELEASE_PURPLE);
        sleep(500);
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
