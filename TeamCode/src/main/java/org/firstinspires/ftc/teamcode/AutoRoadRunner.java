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
 * Extended from AutonomousRight file.
 * Use this one for autonomous when robot located at right side of game field.
 */

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
 *      One camera:
 *          "Webcam 1"
 */

@Autonomous(name="auto RR right", group="Concept")
//@Disabled
public class AutoRoadRunner extends LinearOpMode {

    public int autonomousStartLocation = 1; // 1 for right location, and -1 for left location.

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final SlidersWith2Motors slider = new SlidersWith2Motors();
    private final ArmClawUnit armClaw = new ArmClawUnit();

    private SampleMecanumDrive drive;

    // camera and sleeve color
    ObjectDetection.ParkingLot myParkingLot = ObjectDetection.ParkingLot.UNKNOWN;
    double parkingLotDis = 0;
    ObjectDetection coneSleeveDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;

    // road runner variables
    Pose2d startPose = new Pose2d(-6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH, // -65.0
            -3 * Params.HALF_MAT, Math.toRadians(-90));
    double dropOffAngle = Math.toRadians(26.565 - 90); // actan(1/2) the target robot heading angle when drop off cone
    double dropOffAngle2 = Math.toRadians(26.565 - 90); // the target robot heading angle when drop off cone
    double armX = Params.ARM_UNLOADING_EXTENSION * Math.cos(dropOffAngle);
    double armY = Params.ARM_UNLOADING_EXTENSION * Math.sin(dropOffAngle);

    double armX2 = Params.ARM_UNLOADING_EXTENSION * Math.cos(dropOffAngle2);
    double armY2 = Params.ARM_UNLOADING_EXTENSION * Math.sin(dropOffAngle2);

    Pose2d poseLineEnd1 = new Pose2d(startPose.getX() + 2 * Params.HALF_MAT, startPose.getY(), startPose.getHeading());
    Pose2d poseSplineEnd2 = new Pose2d(-2 * Params.HALF_MAT, startPose.getY(), startPose.getHeading());
    Pose2d poseSplineEnd3 = new Pose2d(-2 * Params.HALF_MAT + 5, -2 * Params.HALF_MAT - 9, dropOffAngle);

    Pose2d poseMJDropOff = new Pose2d(-2 * Params.HALF_MAT + armX - 2.5, -2 * Params.HALF_MAT + armY + 2, dropOffAngle);
    Vector2d VectorMJDropOffEst = new Vector2d(-2 * Params.HALF_MAT + armX, -2 * Params.HALF_MAT + armY);

    Pose2d poseMidPoint1 = new Pose2d(-Params.HALF_MAT, -3 * Params.HALF_MAT, Math.toRadians(-90));
    Pose2d poseMidPoint2 = new Pose2d(-Params.HALF_MAT, -4 * Params.HALF_MAT, Math.toRadians(-90));

    Pose2d poseConeStack = new Pose2d(-Params.HALF_MAT + 1, -6 * Params.HALF_MAT + Params.FLIP_ARM_LENGTH, Math.toRadians(-90));
    Vector2d vectorConeStackEst = new Vector2d(-Params.HALF_MAT, -6 * Params.HALF_MAT + Params.FLIP_ARM_LENGTH);

    Pose2d poseMJDropOff2 = new Pose2d(-2 * Params.HALF_MAT + armX2, -2 * Params.HALF_MAT + armY2, dropOffAngle2);

    Trajectory traj1;

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

        // init drive with road runner
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        slider.init(hardwareMap, "RightSlider", "LeftSlider");
        // Reset slider motor encoder counts kept by the motor
        slider.resetEncoders();

        armClaw.init(hardwareMap, "ArmServo", "ClawServo");
        armClaw.armFlipFrontLoad();
        sleep(2500);
        armClaw.clawClose();
        sleep(200);
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
        drive.setPoseEstimate(startPose);

        // move this code here to save the path build time during autonomous.
        traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseLineEnd1)
                .addDisplacementMarker(Params.HALF_MAT, () -> {
                    // lift slider
                    slider.setInchPosition(Params.MEDIUM_JUNCTION_POS);
                    //armClaw.armFlipBackUnload();
                    armClaw.armFlipBackUnloadPre();
                })
                .splineToSplineHeading(poseSplineEnd2, Math.toRadians(0))
                .splineToSplineHeading(poseSplineEnd3, Math.toRadians(70))
                .splineToLinearHeading(poseMJDropOff, Math.toRadians(70))
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

        // drop cone and back to the center of mat
        drive.setPoseEstimate(new Pose2d(VectorMJDropOffEst, drive.getPoseEstimate().getHeading())); // reset orientation.

        rrUnloadCone();

        for(int autoLoop = 0; autoLoop < 5; autoLoop++) {
            moveFromJunctionToConeStack();

            // load cone
            rrLoadCone(Params.coneStack5th - Params.coneLoadStackGap * autoLoop - 1.0);

            moveFromConeStackToJunction();

            // unload cone & adjust
            rrUnloadCone();

            telemetry.addData("RR", "traj1 end x = %.2f,  y = %.2f, angle = %.2f",
                    traj1.end().getX(), traj1.end().getY(), Math.toDegrees(traj1.end().getHeading()));
            telemetry.addData("RR", "estimate end x = %.2f,  y = %.2f, angle = %.2f",
                    drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.update();
        }

        // parking
        driveToParkingLot(coneSleeveDetect.getParkingLot());

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

        drive.setPoseEstimate(new Pose2d(vectorConeStackEst, drive.getPoseEstimate().getHeading())); // reset orientation

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
        sleep(50); // wait arm flip down a little bit to junction

        //driveForward(Params.DISTANCE_DROP_OFF);
        Logging.log("Wait before unloading claw open.");
        armClaw.clawOpen();
        sleep(Params.CLAW_OPEN_SLEEP); // 50
        armClaw.armFlipFrontLoad();
        Logging.log("Auto unload - Cone has been unloaded.");
    }

    private void moveFromJunctionToConeStack() {
        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseConeStack)
                .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                    slider.setInchPosition(Params.WALL_POSITION);
                })

                // path option 2


                .build();
        drive.followTrajectory(traj1);

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

    private void moveFromConeStackToJunction() {
        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseMJDropOff2)
                .build();
        drive.followTrajectory(traj1);

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

    private void driveToParkingLot(ObjectDetection.ParkingLot parkingLot) {
        double parkingY, parkingX, parkingH;
        Pose2d poseParking;

        // build trajectory
        switch (parkingLot) {
            case LEFT:
                parkingY = -Params.HALF_MAT;
                parkingX = -1 * Params.HALF_MAT;
                parkingH = Math.toRadians(-0);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                            // lower slider
                            slider.setInchPosition(Params.WALL_POSITION);
                        })
                        .build();

                break;
            case CENTER:
                parkingY = -3 * Params.HALF_MAT;
                parkingX = -1 * Params.HALF_MAT;
                parkingH = Math.toRadians(-180);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                // parking
                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                            // lower slider
                            slider.setInchPosition(Params.WALL_POSITION);
                        })
                        .build();
                break;
            case RIGHT:
                parkingY = -5 * Params.HALF_MAT;
                parkingX = -1 * Params.HALF_MAT;
                parkingH = Math.toRadians(-180);
                poseParking = new Pose2d(parkingX, parkingY, parkingH);

                // parking
                traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseParking)
                        .addDisplacementMarker(Params.UNLOAD_DS_VALUE, () -> {
                            // lower slider
                            slider.setInchPosition(Params.WALL_POSITION);
                        })
                        .build();
                break;
        }

        //drving to parking lot
        drive.followTrajectory(traj1);
    }

    /**
     * Set robot starting position: 1 for right and -1 for left.
     */
    public void setRobotLocation() {
        autonomousStartLocation = 1;
    }

    private void driveBack(double distanceInch) {
        Trajectory trajBack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(distanceInch)
                .build();
        drive.followTrajectory(trajBack);
    }

    private void driveForward(double distanceInch) {
        Trajectory trajBack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(distanceInch)
                .build();
        drive.followTrajectory(trajBack);
    }
}
