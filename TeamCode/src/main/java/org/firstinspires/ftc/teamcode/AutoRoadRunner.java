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
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
 *      Tow slider motors:
 *          "RightSlider"
 *          "LeftSlider"
 *
 *      Tow servo motors:
 *          "ArmServo"
 *          "ClawServo"
 *
 *      Two camera:
 *          "Webcam 1"
 *          "WebcamR"
 */

@Autonomous(name="Auto RR right", group="Concept")
//@Disabled
public class AutoRoadRunner extends LinearOpMode {

    // calibration parameters for home Mat.
    Vector2d preConeDropAdjust = new Vector2d(-1.0, 0);
    Vector2d poseConeStackAdjust = new Vector2d(0, 0);
    Vector2d poseMJDropOffAdjust = new Vector2d(0.5, 0.5);
    Vector2d poseHJDropOffAdjust = new Vector2d(0, 0);


    public int startLoc = 1; // 1 for right location, and -1 for left location.

    boolean debug_flag = false;

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
    Pose2d poseHJDropOff;
    Vector2d vHJDropOffEst;

    Trajectory traj1;

    private void setPoses() {
        // road runner variables
        startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH, // -65.0
                -3 * Params.HALF_MAT * startLoc, Math.toRadians(-90 * startLoc));
        dropOffAngle = Math.toRadians(-60 * startLoc);
        dropOffAngle2 = Math.toRadians(-60 * startLoc); // the target robot heading angle when drop off cone

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

        poseHJDropOff = new Pose2d(armXHJ - poseHJDropOffAdjust.getX(),
                -2 * Params.HALF_MAT * startLoc + armYHJ + poseHJDropOffAdjust.getY() * startLoc,
                dropOffAngleHJ);
        vHJDropOffEst = new Vector2d(armXHJ, -2 * Params.HALF_MAT * startLoc + armYHJ);

    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");
        // init drive with road runner
        drive = new SampleMecanumDrive(hardwareMap);

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

    public void autonomousCore() {

        // drive to medium junction, lift sliders, arm to back
        drive.followTrajectory(traj1);

        // for testing
        if (gamepad1.a || gamepad1.b) {
            sleep(5000);
            if (gamepad1.b)
                return;
        }

        // drop cone and back to the center of mat
        drive.setPoseEstimate(new Pose2d(vPreConeDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation.

        rrUnloadCone();

        for(int autoLoop = 0; autoLoop < 5; autoLoop++) {
            moveFromJunctionToConeStack();

            // for testing
            if (gamepad1.a || gamepad1.b) {
                sleep(5000);
                if (gamepad1.b)
                    return;
            }

            drive.setPoseEstimate(new Pose2d(vConeStackEst, drive.getPoseEstimate().getHeading())); // reset orientation

            // load cone
            rrLoadCone(Params.coneStack5th - Params.coneLoadStackGap * autoLoop - 0.5);

            moveFromConeStackToJunction();
            drive.setPoseEstimate(new Pose2d(vMJDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation

            /*
            if (autoLoop < 4) {
                moveFromConeStackToJunction();
                drive.setPoseEstimate(new Pose2d(vMJDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation
            }
            else {
                slider.setInchPosition(Params.HIGH_JUNCTION_POS);
                moveFromConeStackToHJunction();
                drive.setPoseEstimate(new Pose2d(vHJDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation
            }
             */

            // for testing
            if (gamepad1.a || gamepad1.b) {
                sleep(5000);
                if (gamepad1.b)
                    return;
            }

            drive.setPoseEstimate(new Pose2d(vMJDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation
            // unload cone & adjust
            rrUnloadCone();

            if (debug_flag) {
                telemetry.addData("RR", "traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                        traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
                telemetry.addData("RR", "estimate end x = %.2f,  y = %.2f, angle = %.2f",
                        drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
                telemetry.update();
            }
        }

        // parking
        driveToParkingLot(coneSleeveDetect.getParkingLot());


        // storage robot pose of the end of autonomous
        Params.currentPose = drive.getPoseEstimate();

        // lower sliders
        slider.setInchPosition(Params.WALL_POSITION);

        Logging.log("traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
        Logging.log("estimate end x = %.2f,  y = %.2f, angle = %.2f",
                drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));

        telemetry.addData("RR", "traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
        telemetry.addData("RR", "estimate end x = %.2f,  y = %.2f, angle = %.2f",
                drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.update();
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
        slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
        armClaw.armFlipBackUnloadPre();
        Logging.log("Wait before moving out cone stack.");
        sleep(50); // wait slider lift from cone stack.
    }

    /**
     * auto unload cone with roadrunner support.
     */
    public void rrUnloadCone() {
        armClaw.armFlipBackUnload();
        slider.movingSliderInch(-Params.SLIDER_MOVE_DOWN_POSITION / 2.0);
        sleep(100); // wait arm flip down a little bit to junction

        //driveForward(Params.DISTANCE_DROP_OFF);
        Logging.log("Wait before unloading claw open.");
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // 50
        armClaw.armFlipFrontLoad();
        Logging.log("Auto unload - Cone has been unloaded.");
    }

    public void moveFromJunctionToConeStack() {
        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseConeStack)
                .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                    slider.setInchPosition(Params.WALL_POSITION);
                })

                .build();
        drive.followTrajectory(traj1);

        if (debug_flag) {
            Logging.log("Arrived cone stack");
            Logging.log("traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                    traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
            Logging.log("estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));


            telemetry.addData("RR", "traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                    traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
            telemetry.addData("RR", "estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
        }
    }

    public void moveFromConeStackToJunction() {
        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseMJDropOff)
                .build();
        drive.followTrajectory(traj1);

        if (debug_flag) {
            Logging.log("Arrived junction");
            Logging.log("traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                    traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
            Logging.log("estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));

            telemetry.addData("RR", "traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                    traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
            telemetry.addData("RR", "estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
        }
    }

    public void moveFromConeStackToHJunction() {
        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseHJDropOff)
                .addDisplacementMarker(Params.HALF_MAT, () -> {
                    slider.setInchPosition(Params.HIGH_JUNCTION_POS);
                })
                .build();
        drive.followTrajectory(traj1);
    }

    private void driveToParkingLot(ObjectDetection.ParkingLot parkingLot) {
        double parkingY, parkingX, parkingH;
        Pose2d poseParking;

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
                })
                .build();
        drive.followTrajectory(traj1);

        // arm
        armClaw.armFlipCenter();

        // for testing
        if (gamepad1.a || gamepad1.b) {
            sleep(5000);
            if (gamepad1.b)
                return;
        }

        switch (parkingLot) {
            case LEFT:
                parkingX = -Params.HALF_MAT;
                parkingY = (-3 * startLoc + 2) * Params.HALF_MAT - 1;
                parkingH = Math.toRadians(-90 * startLoc);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .build();
                drive.followTrajectory(traj1);

                parkingX = -3 * Params.HALF_MAT + 2;
                parkingY = (-3 * startLoc + 2) * Params.HALF_MAT - 1;
                parkingH = Math.toRadians(-90 * startLoc);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .build();
                drive.followTrajectory(traj1);

                break;

            case CENTER:
                parkingY = -3 * Params.HALF_MAT * startLoc;
                parkingX = -3 * Params.HALF_MAT + 5;
                parkingH = Math.toRadians(180);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .build();
                drive.followTrajectory(traj1);
                break;

            case RIGHT:
                parkingX = -Params.HALF_MAT;
                parkingY = (-3 * startLoc - 2) * Params.HALF_MAT - 1;
                parkingH = Math.toRadians(-90 * startLoc);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .build();

                drive.followTrajectory(traj1);


                parkingX = -3 * Params.HALF_MAT + 2;
                parkingY = (-3 * startLoc - 2) * Params.HALF_MAT - 1;
                parkingH = Math.toRadians(-90 * startLoc);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .build();
                drive.followTrajectory(traj1);
                break;
        }
    }

    /**
     * Set robot starting position: 1 for right and -1 for left.
     */
    public void setRobotLocation() {
        startLoc = 1;
    }

    private void driveBack(double distanceInch) {
        Trajectory trajBack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(distanceInch)
                .build();
        drive.followTrajectory(trajBack);
    }

}
