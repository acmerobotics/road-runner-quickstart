package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Full Integrated Auto (Kalman + Pure Pursuit)", group = "Autonomous")
public class IntegratedAuto extends LinearOpMode {
    private BHI260IMU imu;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // ✅ Initialize Road Runner Drive
        Pose2d startPose = new Pose2d(0, -72.00, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // ✅ Initialize Sensors (IMU & Distance Sensor)
        imu = hardwareMap.get(BHI260IMU.class, "imu");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // ✅ Initialize Kalman Filter (Sensor Fusion)
        KalmanFilter kalmanFilter = new KalmanFilter(startPose, imu, distanceSensor, hardwareMap);

        // ✅ Initialize Pure Pursuit Controller
//        PurePursuitController purePursuit = new PurePursuitController(drive);

        waitForStart();
        if (isStopRequested()) return;

        // 🎯 **Step 1: Move to Expected Location**
        Pose2d targetPose = new Pose2d(0, -60, Math.toRadians(90));
        telemetry.addData("Target Pose", targetPose.toString());
        Action trajectory = drive.actionBuilder(startPose)
                .lineToY(targetPose.position.y)
                .build();
        Actions.runBlocking(trajectory);

        // 🎯 **Step 2: Use Odometry + IMU for Prediction**
        Pose2d odometryPose = drive.pose;
        kalmanFilter.predict(odometryPose);
        telemetry.addData("Drive Pose", drive.pose.toString());
        telemetry.update();

        Actions.runBlocking(new SleepAction(2));

        // 🎯 **Step 3: If Limelight Sees an AprilTag, Correct Position**
        if (kalmanFilter.correctWithLimelight()) {
            telemetry.addLine("✅ Found April Tag, Getting Pose Estimation");

            // 🎯 **Step 4: Use Distance Sensor for Additional Correction**
//        kalmanFilter.correctWithDistanceSensor();

            // 🎯 **Step 5: Update Road Runner with the Best Estimate**
            Pose2d correctedPose = kalmanFilter.getEstimate();
            drive.pose = correctedPose;
            telemetry.addData("Actual Pose", correctedPose.toString());
            telemetry.addData("Heading", correctedPose.heading.toString());
            telemetry.update();

            Actions.runBlocking(new SleepAction(5));
            // 🎯 **Step 6: Use Pure Pursuit for Next Movement**
            if (!posesMatch(correctedPose, targetPose)) { // If position is off
                telemetry.addLine("🔄 Recalculating Trajectory to Corrected Target");
                telemetry.update();

                // ✅ Generate a new corrected trajectory
                Action correctionTrajectory = drive.actionBuilder(correctedPose)
                        .lineToY(targetPose.position.y)
                        .build();

                // ✅ Follow the newly corrected trajectory
                Actions.runBlocking(correctionTrajectory);
            }
        }

        telemetry.addLine("✅ Navigation Completed");
        telemetry.update();

        Actions.runBlocking(new SleepAction(10));
    }

    public boolean posesMatch(Pose2d pose1, Pose2d pose2){
        return pose1.position.x == pose2.position.x &&
                pose1.position.y == pose2.position.y &&
                pose1.heading.toDouble() == pose2.heading.toDouble();
    }
}
