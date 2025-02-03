package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Common.Limelight;

public class KalmanFilter {
    private double poseX, poseY, heading;
    private final double processNoise = 0.1; // Adjust for smoothness
    private final double measurementNoise = 0.05; // Lower = trust sensors more
    private final double distanceSensorNoise = 0.07; // Lower = trust distance sensor more
    private Limelight limelight;
    private HardwareMap hardwareMap;

    private BHI260IMU imu;
    private DistanceSensor distanceSensor;

    public KalmanFilter(Pose2d initialPose, BHI260IMU imu, DistanceSensor distanceSensor, HardwareMap hwmap) {
        this.poseX = initialPose.position.x;
        this.poseY = initialPose.position.x;
        this.heading = initialPose.heading.toDouble();
        this.imu = imu;
//        this.distanceSensor = distanceSensor;
        this.hardwareMap = hwmap;

        limelight = new Limelight(hwmap);
    }

    /**
     * 🚀 **Prediction Step (Using Odometry & IMU)**
     * Updates the estimated position based on odometry and IMU heading.
     */
    public void predict(Pose2d odometryPose) {
        double imuHeading = getIMUHeading(); // Fetch IMU heading
        poseX = odometryPose.position.x;
        poseY = odometryPose.position.y;
        heading = imuHeading;
    }

    /**
     * 🎯 **Correction Step (Using Limelight AprilTags)**
     * When AprilTags are detected, this corrects the estimated position.
     */
    public boolean correctWithLimelight() {
        limelight.updateLimelight();
        if (limelight.result != null && limelight.result.isValid()) { // If an AprilTag is detected
            double tagX = limelight.metersToInches(limelight.botPose.getPosition().x);
            double tagY = limelight.metersToInches(limelight.botPose.getPosition().y);
            double tagHeading = limelight.botPose.getOrientation().getYaw(AngleUnit.DEGREES); // Assume tag is upright

            // Apply correction with measurement noise weighting
            poseX += (tagX - poseX) * measurementNoise;
            poseY += (tagY - poseY) * measurementNoise;
            heading += (tagHeading - heading) * measurementNoise;
            return true;
        } else {
            return false;
        }
    }

    /**
     * 📏 **Correction Step (Using Distance Sensor)**
     * Adjusts position if distance sensor is within a valid range.
     */
//    public void correctWithDistanceSensor() {
//        double distance = distanceSensor.getDistance(com.qualcomm.robotcore.hardware.DistanceUnit.INCH);
//        if (distance > 0 && distance < 50) { // Only correct if within reasonable range
//            poseY = 72 - distance; // Assuming wall is at y=72
//        }
//    }

    /**
     * 🧭 **IMU Heading Retrieval**
     * Fetches real-time heading from the Control Hub’s IMU.
     */
    private double getIMUHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(); // Fetch IMU heading
    }

    /**
     * 🎯 **Retrieve Best Pose Estimate**
     * Returns the best estimated pose after applying corrections.
     */
    public Pose2d getEstimate() {
        return new Pose2d(poseX, poseY, heading);
    }
}
