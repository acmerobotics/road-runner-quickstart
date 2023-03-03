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
 *      Two slider motors:
 *          "RightSlider"
 *          "LeftSlider"
 *
 *      Two servo motors:
 *          "ArmServo"
 *          "ClawServo"
 *
 *      Two cameras:
 *          "Webcam 1"
 *          "WebcamR"
 */

@Autonomous(name="Auto M right", group="Concept")
//@Disabled
public class AutoMJ_Right extends LinearOpMode {

    // calibration parameters for home Mat - old.
    /*
    Vector2d preConeDropAdjust = new Vector2d(-1, -0.5);
    Vector2d poseConeStackAdjust = new Vector2d(-1, -0.5);
    Vector2d poseMJDropOffAdjust = new Vector2d(0, 0);
     */

    // BC lab mat - new
    /*
    Vector2d preConeDropAdjust = new Vector2d(-2, 1.5);
    Vector2d poseConeStackAdjust = new Vector2d(-0.2, -0.5);
    Vector2d poseMJDropOffAdjust = new Vector2d(1.2, 1.1);
     */

    // average calibration values - good for the math in BC
    Pose2d preConeDropAdjust = new Pose2d(-0.5, 0, 0);
    Pose2d poseConeStackAdjust = new Pose2d(0, -1.5, 0);
    Pose2d poseMJDropOffAdjust = new Pose2d(1, 1, 0);

    double armLengthAdj = 0.0;
    boolean withDW = with2DW || with3DW; // with dead wheels
    boolean compensationOn = true;
    double splineVelocity = 45.0;
    double splineMAX_ACCEL = 45.0;

    Pose2d poseHJPreConAdjust = new Pose2d(0, 0, 0);
    Pose2d poseHJDropOffAdjust = new Pose2d(0, 0, 0);

    public int startLoc = 1; // 1 for right location, and -1 for left location.
    public int junctionType = 1; // 1 for medium junction, 2 for high junction
    boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final SlidersWith2Motors slider = new SlidersWith2Motors();
    private final ArmClawUnit armClaw = new ArmClawUnit();

    public SampleMecanumDrive drive;

    // camera and sleeve color
    ObjectDetection.ParkingLot myParkingLot = ObjectDetection.ParkingLot.UNKNOWN;
    double parkingLotDis = 0;
    ObjectDetection coneSleeveDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;

    // road runner variables
    Pose2d startPose;
    
    // medium junction
    double dropOffAngle, dropOffAngle2;
    double armX, armY;
    double armX2, armY2;
    // high junction
    double dropOffAngleHJ;
    double armXHJ;
    double armYHJ;

    // pre cone to medium junction
    Pose2d poseLineEnd1, poseSplineEnd2, poseSplineEnd3, posePreConeDropOff;
    Vector2d vPreConeDropOffEst;
    // cone stack
    Pose2d poseConeStack;
    Vector2d vConeStackEst;
    // medium junction pose
    Pose2d poseMJDropOff;
    Vector2d vMJDropOffEst;

    // high junction pose
    Pose2d poseHJPrecon;
    Vector2d vHJPreCon;

    Pose2d poseHJDropOff;
    Vector2d vHJDropOffEst;

    Trajectory traj1;
    TrajectorySequence trajSeq1; // use circle path
    TrajectorySequence trajSeq2;
    TrajectorySequence trajSeq3;
    boolean heading0 = false; // the heading angle of start position

    /**
     * Set robot starting position: 1 for right and -1 for left.
     */
    public void setRobotLocation() {
        startLoc = 1;

        // compensationOn parameters
        if (!withDW) {
            preConeDropAdjust = new Pose2d(-1.0, -1.0, 0);
            poseConeStackAdjust = new Pose2d(0, -0.8, 0);
            poseMJDropOffAdjust = new Pose2d(-0.3, 0, 0);
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
        startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH + 1.0, // -64.0
                -3 * Params.HALF_MAT * startLoc, Math.toRadians(-90 * startLoc));
        dropOffAngle = Math.toRadians(-55 * startLoc);
        dropOffAngle2 = Math.toRadians(-55 * startLoc); // the target robot heading angle when drop off cone

        if (compensationOn) {
            startPose = startPose.plus(new Pose2d(- preConeDropAdjust.getX(), - preConeDropAdjust.getY() * startLoc, 0));
        }

        if (heading0) {
            startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2.0 + 1.0, // -64.0
                    -3 * Params.HALF_MAT * startLoc, 0);
        }

        // high junction drop off
        dropOffAngleHJ = Math.toRadians(-120 * startLoc);
        armXHJ = Params.ARM_UNLOADING_EXTENSION * Math.cos(dropOffAngleHJ);
        armYHJ = Params.ARM_UNLOADING_EXTENSION * Math.sin(dropOffAngleHJ);

        armX = Params.ARM_UNLOADING_EXTENSION * Math.cos(dropOffAngle);
        armY = Params.ARM_UNLOADING_EXTENSION * Math.sin(dropOffAngle);

        armX2 = Params.ARM_UNLOADING_EXTENSION * Math.cos(dropOffAngle2);
        armY2 = Params.ARM_UNLOADING_EXTENSION * Math.sin(dropOffAngle2);

        poseLineEnd1 = new Pose2d(startPose.getX() + 2 * Params.HALF_MAT, startPose.getY(), startPose.getHeading());
        poseSplineEnd2 = new Pose2d(-2 * Params.HALF_MAT, startPose.getY(), startPose.getHeading());
        poseSplineEnd3 = new Pose2d(-2 * Params.HALF_MAT + 5, -(2 * Params.HALF_MAT + 9) * startLoc, dropOffAngle);

        posePreConeDropOff = new Pose2d(-2 * Params.HALF_MAT + armX + preConeDropAdjust.getX(),
                -2 * Params.HALF_MAT * startLoc + armY + preConeDropAdjust.getY() * startLoc,
                dropOffAngle);
        vPreConeDropOffEst = new Vector2d(-2 * Params.HALF_MAT + armX, -2 * Params.HALF_MAT * startLoc + armY);

        poseConeStack = new Pose2d(-Params.HALF_MAT + poseConeStackAdjust.getX(),
                (-6 * Params.HALF_MAT + Params.FLIP_ARM_LENGTH + poseConeStackAdjust.getY()) * startLoc,
                Math.toRadians(-90 * startLoc));
        vConeStackEst = new Vector2d(-Params.HALF_MAT, (-6 * Params.HALF_MAT + Params.FLIP_ARM_LENGTH) * startLoc);

        poseMJDropOff = new Pose2d(-2 * Params.HALF_MAT + armX2 + poseMJDropOffAdjust.getX(),
                -2 * Params.HALF_MAT * startLoc + armY2 + poseMJDropOffAdjust.getY() * startLoc, dropOffAngle2);
        vMJDropOffEst = new Vector2d(-2 * Params.HALF_MAT + armX2, -2 * Params.HALF_MAT * startLoc + armY2);

        // high junction
        poseHJPrecon  = new Pose2d(armXHJ + poseHJPreConAdjust.getX(),
                -2 * Params.HALF_MAT * startLoc + armYHJ + poseHJPreConAdjust.getY() * startLoc,
                dropOffAngleHJ);
        vHJPreCon = new Vector2d(armXHJ, -2 * Params.HALF_MAT * startLoc + armYHJ);

        poseHJDropOff = new Pose2d(armXHJ + poseHJDropOffAdjust.getX(),
                -2 * Params.HALF_MAT * startLoc + armYHJ + poseHJDropOffAdjust.getY() * startLoc,
                dropOffAngleHJ);
        vHJDropOffEst = new Vector2d(armXHJ, -2 * Params.HALF_MAT * startLoc + armYHJ);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setRobotLocation();

        setPoses();

        // camera for sleeve color detect, start camera at the beginning.
        if (startLoc > 0) {
            webcamName = "Webcam 1";
        }
        else {
            webcamName = "WebcamR";
        }
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

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slider.init(hardwareMap, "RightSlider", "LeftSlider");
        // Reset slider motor encoder counts kept by the motor
        slider.resetEncoders();

        armClaw.init(hardwareMap, "ArmServo", "ClawServo");
        armClaw.armFlipFrontLoad();
        sleep(500);
        armClaw.clawClose();
        sleep(500);
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
            telemetry.addData("robot position: ", startLoc > 0? "Right":"Left");
            telemetry.addData("RR", "imu Heading = %.1f",
                    Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // move this code here to save the path build time during autonomous.
        if (1 == junctionType) {

            // medium junction
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseLineEnd1)
                    .addDisplacementMarker(Params.HALF_MAT, () -> {
                        // lift slider
                        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
                        armClaw.armFlipBackUnloadPre();
                    })
                    .splineToSplineHeading(poseSplineEnd2, Math.toRadians(0))
                    .splineToSplineHeading(poseSplineEnd3, Math.toRadians(70 * startLoc))
                    .splineToLinearHeading(posePreConeDropOff, Math.toRadians(70 * startLoc))
                    .build();

            trajSeq2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(splineVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(splineMAX_ACCEL))
                    .lineToLinearHeading(poseLineEnd1)
                    .addDisplacementMarker(Params.HALF_MAT, () -> {
                        // lift slider
                        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
                        armClaw.armFlipBackUnloadPre();
                    })
                    .splineToSplineHeading(poseSplineEnd2, Math.toRadians(0))
                    .splineToSplineHeading(poseSplineEnd3, Math.toRadians(70 * startLoc))
                    .splineToLinearHeading(posePreConeDropOff, Math.toRadians(70 * startLoc))
                    .resetConstraints()
                    .build();
            posePreConeDropOff = setMJPreConePath();
            if (heading0) {
                posePreConeDropOff = setMJPreConePath3();
            }
        }

        if (2 == junctionType) {
            // high junction
            poseSplineEnd3 = new Pose2d(-8, startPose.getY(), dropOffAngleHJ);
            traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(poseLineEnd1)
                    .addDisplacementMarker(2 * Params.HALF_MAT, () -> {
                        // lift slider
                        slider.setInchPosition(Params.HIGH_JUNCTION_POS);
                        armClaw.armFlipBackUnloadPre();
                    })
                    .splineToSplineHeading(poseSplineEnd2, Math.toRadians(0))
                    .splineToSplineHeading(poseSplineEnd3, Math.toRadians(30))
                    .splineToLinearHeading(poseHJPrecon, Math.toRadians(30))
                    .build();
        }

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            autonomousCore();
            camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
            Logging.log("Autonomous - total Run Time: " + runtime);
        }
        // The motor stop on their own but power is still applied. Turn off motor.
        slider.stop();
    }

    public void autonomousCore() {

        double coneNum = 5;
        if (2 == junctionType) {
            coneNum = 4;
        }

        // drive to medium junction, lift sliders, arm to back
        Logging.log("Autonomous - Starting raj1: " + runtime);

        if (heading0) {
            drive.followTrajectorySequence(trajSeq1);
        }
        else {
            drive.followTrajectory(traj1);
            //drive.followTrajectorySequence(trajSeq2);
        }
        Logging.log("Autonomous - ending traj1: " + runtime);

        if (debug_flag) {
            Logging.log("Arrived pre cone junction");
            Logging.log("start pose: end x = %.2f,  y = %.2f, angle = %.2f",
                    startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));

            Logging.log("pre-cone pose trajSeq1: end x = %.2f,  y = %.2f, angle = %.2f",
                    trajSeq1.end().getX(), trajSeq1.end().getY(), Math.toDegrees(trajSeq1.end().getHeading()));

            Logging.log("pre-cone pose trajSeq2: end x = %.2f,  y = %.2f, angle = %.2f",
                    trajSeq2.end().getX(), trajSeq2.end().getY(), Math.toDegrees(trajSeq2.end().getHeading()));

            Logging.log("pre-cone pose traj1: end x = %.2f,  y = %.2f, angle = %.2f",
                    traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));

            Logging.log("pre-cone pose estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
        }

        // for testing
        if (gamepad1.a || gamepad1.b) {
            sleep(2000);
            if (gamepad1.b)
                return;
        }

        // drop cone and back to the center of mat
        if (compensationOn) {
            if (1 == junctionType) {
                drive.setPoseEstimate(new Pose2d(vPreConeDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation.
            }

            if (2 == junctionType) {
                drive.setPoseEstimate(new Pose2d(vHJPreCon, drive.getPoseEstimate().getHeading())); // reset orientation.
            }
        }

        rrUnloadCone();
        //drive.setPoseEstimate(posePreConeDropOff);

        for(int autoLoop = 0; autoLoop < coneNum; autoLoop++) {

            moveFromJunctionToConeStack(Params.WALL_POSITION - Params.coneLoadStackGap * autoLoop);

            // for testing
            if (gamepad1.a || gamepad1.b) {
                sleep(2000);
                if (gamepad1.b)
                    return;
            }
            if (compensationOn) {
                drive.setPoseEstimate(new Pose2d(vConeStackEst, drive.getPoseEstimate().getHeading())); // reset orientation
            }
            // load cone
            rrLoadCone(Params.coneStack5th - Params.coneLoadStackGap * autoLoop - 0.5);

            if (1 == junctionType) {
                moveFromConeStackToJunction();
                if (compensationOn) {
                    drive.setPoseEstimate(new Pose2d(vMJDropOffEst, drive.getPoseEstimate().getHeading()));
                }
            }
            else if (2 == junctionType) {
                moveFromConeStackToHJunction();
                if (compensationOn) {
                    drive.setPoseEstimate(new Pose2d(vHJDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation
                }
            }

            // for testing
            if (gamepad1.a || gamepad1.b) {
                sleep(2000);
                if (gamepad1.b)
                    return;
            }

            // unload cone & adjust
            rrUnloadCone();
        }

        // parking
        driveToParkingLot(myParkingLot);

        // storage robot pose of the end of autonomous
        Params.currentPose = drive.getPoseEstimate();

        Logging.log("estimate end x = %.2f,  y = %.2f, angle = %.2f",
                drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
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
        sleep((coneLocation < Params.coneLoadStackGap)? Params.CLAW_CLOSE_SLEEP + 100 : Params.CLAW_CLOSE_SLEEP);// 100ms additional for last cone
        if (1 == junctionType) {
            slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
        }
        else {
            slider.setInchPosition(Params.HIGH_JUNCTION_POS);
        }
        armClaw.armFlipBackUnloadPre();
        Logging.log("Wait before moving out cone stack.");
        sleep(50); // wait slider lift from cone stack.
    }

    /**
     * auto unload cone with roadrunner support.
     */
    public void rrUnloadCone() {
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION / 2.0);
        armClaw.armFlipBackUnload();
        slider.waitRunningComplete();
        //sleep(100); // wait arm flip down a little bit to junction
        Logging.log("Wait before unloading claw open.");
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // 50
        armClaw.armFlipFrontLoad();
        Logging.log("Auto unload - Cone has been unloaded.");
    }

    /**
     * Drop point to stack
     */
    public void moveFromJunctionToConeStack(double coneLoc) {
        //drop point to stack
        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(splineVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(splineMAX_ACCEL))
                .splineTo(poseConeStack.vec(), poseConeStack.getHeading())
                .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                    slider.setInchPosition(coneLoc);
                })
                .resetConstraints()
                .build();
        drive.followTrajectorySequence(traj);

        if (debug_flag) {
            Logging.log("Arrived cone stack");
            Logging.log("traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                    traj.end().getX(), traj.end().getY(), Math.toDegrees(traj.end().getHeading()));
            Logging.log("estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));;
        }
    }

    /**
     * cone stack to medium junction
     */
    public void moveFromConeStackToJunction() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(splineVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(splineMAX_ACCEL))
                .splineTo(poseMJDropOff.vec(), poseMJDropOff.getHeading() + Math.PI)
                .resetConstraints()
                .build();
        drive.followTrajectorySequence(traj);

        if (debug_flag) {
            Logging.log("Arrived junction");
            Logging.log("traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                    traj.end().getX(), traj.end().getY(), Math.toDegrees(traj.end().getHeading()));
            Logging.log("estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));

            Logging.log("MJ pos est x= %.2f", poseMJDropOffAdjust.getX());
            Logging.log("poseMJDropOff x= %.2f", poseMJDropOff.getX());
        }
    }

    public void moveFromConeStackToHJunction() {
        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseHJDropOff)
                .build();
        drive.followTrajectory(traj1);
    }

    private void driveToParkingLot(ObjectDetection.ParkingLot parkingLot) {
        double parkingY, parkingX, parkingH;
        Pose2d poseParking;

        TrajectorySequence parkingt;

        // move to 2nd mat center, which is green parking lot
        parkingX = -Params.HALF_MAT;
        parkingY = -3 * Params.HALF_MAT * startLoc;
        parkingH = Math.toRadians(-90 * startLoc);
        Pose2d poseParkingGreen = new Pose2d(parkingX, parkingY, parkingH);

        // parking
        traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseParkingGreen)
                .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                    // lower slider
                    slider.setInchPosition(Params.WALL_POSITION);
                    armClaw.armFlipCenter();
                })
                .build();

        switch (parkingLot) {
            case LEFT:
                parkingX = -Params.HALF_MAT;
                parkingY = (-3 * startLoc + 2) * Params.HALF_MAT - 1;
                parkingH = Math.toRadians(-90 * startLoc);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                if (startLoc < 0) {
                    TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineTo(poseConeStack.vec(), poseConeStack.getHeading())
                            .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                                slider.setInchPosition(Params.WALL_POSITION);
                                armClaw.armFlipCenter();
                            })
                            .build();
                    drive.followTrajectorySequence(traj);
                }
                else {
                    drive.followTrajectory(traj1); // move to center

                    traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(poseParking)
                            .build();
                    drive.followTrajectory(traj1);
                }
                break;

            case CENTER:
                parkingX = -3 * Params.HALF_MAT;
                parkingY = -3 * Params.HALF_MAT * startLoc;
                parkingH = Math.toRadians(180);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                parkingt = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addTrajectory(traj1)
                        .build();
                drive.followTrajectorySequence(parkingt);
                break;

            case RIGHT:
                parkingX = -Params.HALF_MAT;
                parkingY = (-3 * startLoc - 2) * Params.HALF_MAT + 1;
                parkingH = Math.toRadians(-90 * startLoc);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                if (startLoc > 0) {
                    TrajectorySequence traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineTo(poseConeStack.vec(), poseConeStack.getHeading())
                            .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                                // lower slider
                                slider.setInchPosition(Params.WALL_POSITION);
                                armClaw.armFlipCenter();
                            })
                            .build();
                    drive.followTrajectorySequence(traj);
                }
                else {
                    drive.followTrajectory(traj1); // move to center

                    traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(poseParking)
                            .build();
                    drive.followTrajectory(traj1);
                }

                break;
        }
        Logging.log("Complete parking.");
    }

    private void driveBack(double distanceInch) {
        Trajectory trajBack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(distanceInch)
                .build();
        drive.followTrajectory(trajBack);
    }

    private Pose2d setMJPreConePath() {

        // check the detail field coordinate system and labels in doc/FieldCoordinateSystem.pdf file
        double armLength = Params.ARM_UNLOADING_EXTENSION + armLengthAdj; // in inch
        double alpha = 90.0 + Math.toDegrees(dropOffAngle);
        double alphaR = Math.toRadians(alpha);
        double beta = (90.0 - alpha) / 2.0;
        double betaR = Math.toRadians(beta);

        // add calculate equations here
        double eg = Params.HALF_MAT;
        double cg = eg / Math.cos(alphaR);
        double ag = armLength;
        double ac = cg - ag;
        double ao = ac * Math.tan(betaR);
        double bo = ao;
        double fo = ao;
        double ax = armLength * Math.sin(alphaR);
        double gx = armLength * Math.cos(alphaR);
        double bc = ac;
        double ce = eg * Math.tan(alphaR);
        double be = ce - bc;
        double co = ac / Math.cos(betaR);
        double cVx = -eg * 2 + ce;
        double cf = co - fo;
        double fh = cf * bo / co;
        double ch = cf * bc / co;

        // update below 6 variable values
        double Ax = ax - eg * 2;
        double Ay = -eg * 2 - gx;
        double Fx = cVx - ch;
        double Fy = fh - eg * 3;
        double Bx = be - eg * 2;
        double By = -eg * 3;

        Pose2d bB = new Pose2d(Bx, By * startLoc, Math.toRadians(-90.0) * startLoc);
        Pose2d fF = new Pose2d(Fx, Fy * startLoc, Math.toRadians(-90.0 + alpha / 2.0) * startLoc);
        Pose2d aA = new Pose2d(Ax, Ay * startLoc, Math.toRadians(-90.0 + alpha) * startLoc);

        trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(splineVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(splineMAX_ACCEL))
                .strafeLeft(24.0 * startLoc)
                .addDisplacementMarker(Params.HALF_MAT / 2.0, () -> {
                    // lift slider
                    camera.closeCameraDevice();

                    slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
                    armClaw.armFlipBackUnloadPre();
                })
                .splineToSplineHeading(bB, 0.0)
                .splineToSplineHeading(fF, Math.toRadians(90.0 + alpha) / 2.0)
                .splineToSplineHeading(aA, Math.toRadians(90.0 +  alpha))
                .resetConstraints()
                .build();

        if (debug_flag) {
            Logging.log("Arrived pre cone junction");
            Logging.log("pre-cone pose: end x = %.2f,  y = %.2f, angle = %.2f",
                    trajSeq1.end().getX(), trajSeq1.end().getY(), Math.toDegrees(trajSeq1.end().getHeading()));
        }
        return aA;
    }

    private Pose2d setMJPreConePath3() {
        Pose2d p1 = new Pose2d(-Params.CHASSIS_LENGTH / 2.0 - 3.0, startPose.getY(), startPose.getHeading());
        trajSeq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(p1.vec())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(splineVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(splineMAX_ACCEL))
                .addDisplacementMarker(Params.HALF_MAT, () -> {
                    // lift slider
                    //camera.closeCameraDevice();
                    slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
                    armClaw.armFlipBackUnloadPre();
                })
                //.waitSeconds(0.050)
                //.splineToSplineHeading(bB, 0.0)
                //.splineToSplineHeading(fF, Math.toRadians(90.0 + alpha) / 2.0)
                //.splineToSplineHeading(aA, Math.toRadians(90.0 +  alpha))
                .setReversed(true)
                .splineTo(posePreConeDropOff.vec(), posePreConeDropOff.getHeading() + Math.PI)
                .resetConstraints()
                .build();

        return trajSeq1.end();
    }
}
