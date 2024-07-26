/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our;
//other imports

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 */
//John - starts program and creates variables
@Autonomous(name="AutoBlueCloseFullApril", group="Robot")

@Disabled

public class ZAutoBlueCloseFullApril extends LinearOpMode {

    // John - Starts a timer so you know how long the program has been running
    private ElapsedTime runtime = new ElapsedTime();
    private  ElapsedTime teamproptime = new ElapsedTime();

    private ElapsedTime runtimemotor = new ElapsedTime();
    private ElapsedTime wait = new ElapsedTime();

    // John - creates variables for all 4 motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo claw;
    private DcMotor shoulder;
    //camera stuff
    double angle = 0;
    double distance = 0;
    double speedmultiply = 0.35;
    double time = 0;
    double wheelmaxspeedrpm = 312;
    double wheelradius = 48;
    double wheelmaxspeedcmps = (((2 * Math.PI * wheelradius)/60) * (wheelmaxspeedrpm))/20000;
    double error = 1;
    double integralmultiply = .4;
    IMU imu;
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;
    int gain = 2;
    NormalizedRGBA normalizedColors;
    int color;
    float hue;
    float saturation;
    float value;
    private ColorSensor daLight;
    private LED red;
    private LED green;
    boolean prophere;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    boolean apriltagvisible = false;
    double xpose = 0;
    int id = 0;
    double leftorright;
    double shoulderStick;
    int shoulder_Position;
    int tagnumber;
    @Override
    public void runOpMode() {

        initializewheels();
        initializeimu();
        initializecolorsensor();
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        angle = 0;
        distance = 35;
        speedmultiply = 0.35;
        movedistancetime();
        angle = 90;
        distance = 10;
        movedistancetime();
        checkforteampropblue();
        if (prophere) {
            angle = 0;
            distance = -10;
            movedistancetime();
            angle = -90;
            distance = 35;
            movedistancetime();
            angle = -90;
            speedmultiply = 0.2;
            turnanglegyro();

            tagnumber = 3;
            leftorright = 1;
            moveuntilapriltag();

            shoulder_Position = shoulder.getCurrentPosition();
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setTargetPosition(shoulder_Position-200);

            //move to wall
            releasepixel();
            time = 1;
            waitfor();
            angle = 0;
            distance = -10;
            speedmultiply = 0.1;
            movedistancetime();
        }
        else {
            angle = -90;
            distance = 25;
            movedistancetime();
            checkforteampropblue();
            if (prophere){
                angle = 0;
                distance = -10;
                movedistancetime();
                angle = -90;
                distance = 10;
                movedistancetime();
                angle = -90;
                speedmultiply = 0.2;
                turnanglegyro();


                tagnumber = 1;
                leftorright = 1;
                moveuntilapriltag();

                shoulder_Position = shoulder.getCurrentPosition();
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setTargetPosition(shoulder_Position-200);
                shoulder.setPower(1);

                //move to wall
                releasepixel();
                time = 1;
                waitfor();
                angle = 0;
                distance = -10;
                speedmultiply = 0.1;
                movedistancetime();
            }
            else {
                angle = 90;
                distance = 15;
                movedistancetime();
                angle = 0;
                distance = 15;
                movedistancetime();
                distance = -25;
                movedistancetime();
                angle = -90;
                distance = 25;
                movedistancetime();
                angle = -90;
                speedmultiply = 0.2;
                turnanglegyro();

                tagnumber = 2;
                leftorright = 1;
                moveuntilapriltag();

                shoulder_Position = shoulder.getCurrentPosition();
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shoulder.setTargetPosition(shoulder_Position-200);
                shoulder.setPower(1);

                //move to wall
                releasepixel();
                time = 1;
                waitfor();
                angle = 0;
                distance = -10;
                speedmultiply = 0.1;
                movedistancetime();
            }
        }
        closeclaw();
        angle = 90;
        speedmultiply = 0.2;
        turnanglegyro();
        imu.resetYaw();

    }
    private  void releasepixel (){
        claw.setPosition(0);
    }
    private  void closeclaw (){
        claw.setPosition(1);
    }
    private void waitfor() {
        wait.reset();
        while (wait.seconds() < time){
            telemetry.addData("Status", "waiting");
        }
    }
    private void movedistancetime () {
        if (opModeIsActive()) {
            double max;
            double axial = 90 - abs(angle);
            double lateral = 90 - abs(angle - 90);
            leftFrontPower = distance * speedmultiply * (axial - lateral);
            rightFrontPower = distance * speedmultiply * (axial + lateral);
            leftBackPower = distance * speedmultiply * (axial + lateral );
            rightBackPower = distance * speedmultiply * (axial - lateral);
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > speedmultiply) {
                double conversion = speedmultiply/max;
                leftFrontPower = leftFrontPower * conversion;
                rightFrontPower = rightFrontPower * conversion;
                leftBackPower = leftBackPower * conversion;
                rightBackPower = rightBackPower * conversion;
            }
            runtimemotor.reset();
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
            while ((opModeIsActive()) && (runtimemotor.milliseconds() * speedmultiply * wheelmaxspeedcmps < abs(distance))) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("LoopRunTime", "time",  runtimemotor.milliseconds());
                telemetry.update();
            }
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
    }
    private void turnanglegyro () {
        if(opModeIsActive()){
            double previoustime = 0;
            double turnspeed;
            double integral = 0;
            error = 1;
            imu.resetYaw();
            integralmultiply = speedmultiply;
            runtimemotor.reset();
            while ((opModeIsActive()) && (abs(error) >= 0.1)){
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                error = (angle+orientation.getYaw(AngleUnit.DEGREES));
                integral = integral + ((error/70)*(runtimemotor.seconds() - previoustime));
                turnspeed = (speedmultiply * (error/70 + (integral * integralmultiply)));
                leftFrontPower = turnspeed;
                rightFrontPower = -turnspeed;
                leftBackPower = turnspeed;
                rightBackPower = -turnspeed;

                leftFrontDrive.setPower(leftFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightFrontDrive.setPower(rightFrontPower);
                rightBackDrive.setPower(rightBackPower);
                telemetry.addData("error", error);
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.update();
                previoustime = runtimemotor.seconds();
            }
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            telemetry.addData("Status", "done");
            telemetry.update();
        }
    }
    private void initializewheels() {
        //John - connects the variable to the actual motor. This makes the variable control the
        //       wheel movement. Make sure the device name is what you set them on the robot
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FRONT_L");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BACK_L");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FRONT_R");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BACK_R");


        //John - sets the direction the wheels move, change these to make the robot move the right way
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        shoulder = hardwareMap. get(DcMotor.class, "shoulder");
        claw = hardwareMap. get(Servo.class, "claw");
        claw.setPosition(1);
        claw.scaleRange(0, 1);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void initializeimu() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    private  void initializecolorsensor() {
        daLight = hardwareMap.get(ColorSensor.class, "daLight");
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //builder.setCamera(BuiltinCameraDirection.BACK);

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private void checkforteampropred(){
        teamproptime.reset();
        prophere = false;
        while (opModeIsActive() && teamproptime.seconds() < 1){
            ((NormalizedColorSensor)daLight).setGain(gain);
            normalizedColors = ((NormalizedColorSensor) daLight).getNormalizedColors();
            color = normalizedColors.toColor();
            hue = JavaUtil.colorToHue(color);
            if (hue <= 50) {
                prophere = true;
            }
        }
    }
    private void checkforteampropblue(){
        teamproptime.reset();
        prophere = false;
        while (opModeIsActive() && teamproptime.seconds() < 1){
            ((NormalizedColorSensor)daLight).setGain(gain);
            normalizedColors = ((NormalizedColorSensor) daLight).getNormalizedColors();
            color = normalizedColors.toColor();
            hue = JavaUtil.colorToHue(color);
            if (hue <= 350 && hue >= 200) {
                prophere = true;
            }
        }
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.update();
        //telemetry.addData();
    }
    private  void apriltagdata() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == tagnumber){
                apriltagvisible = true;
                xpose = detection.ftcPose.x;
                id = detection.id;
                speedmultiply = 0.2;
            }
            else {
                apriltagvisible = false;
                xpose = 2;
                id = 0;
                speedmultiply = 0.35;
            }
        }
    }
    private void moveuntilapriltag(){
        while ((opModeIsActive()) && (id == tagnumber) && (apriltagvisible = true) && (xpose < 2)) {
            angle = leftorright * 90;
            distance = xpose;
            movedistancetime();
            apriltagdata();
        }
    }
    private void movetowall() {
        int sensordistance = 20;
        while (sensordistance >= 5){
            sensordistance = sensordistance;
            angle = 0;
            distance = 1;
            speedmultiply = 0.25;
            movedistancetime();
        }
    }
}