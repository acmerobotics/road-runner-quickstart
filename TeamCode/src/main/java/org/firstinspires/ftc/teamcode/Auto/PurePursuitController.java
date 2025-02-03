package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class PurePursuitController {
    private final MecanumDrive drive;
    private final double lookaheadDistance = 8.0; // Tune this parameter for smoothness

    public PurePursuitController(MecanumDrive drive) {
        this.drive = drive;
    }

    /**
     * 📌 **Follows a smooth trajectory using Pure Pursuit**
     * Instead of stopping at waypoints, the robot dynamically adjusts
     * towards a moving "lookahead" point for continuous motion.
     */
    public void followPath(Pose2d targetPose) {
//        while (!drive.bu()) {
//            Pose2d currentPose = drive.pose;
//            Vector2d lookaheadPoint = getLookaheadPoint(currentPose, targetPose);
//
//            // Compute steering correction
//            double angleError = Math.atan2(lookaheadPoint.y - currentPose.position.y,
//                    lookaheadPoint.x - currentPose.position.x);
//
//            double turnPower = angleError * 0.1; // Tune this gain for accuracy
//            drive.setWeightedDrivePower(new Pose2d(0.5, 0, turnPower));
//
//            drive.updatePoseEstimate();
//        }
    }

    /**
     * 📌 **Determines the Lookahead Point for Smooth Tracking**
     * - Ensures the robot always moves towards a point slightly ahead of it.
     */
    private Vector2d getLookaheadPoint(Pose2d current, Pose2d target) {
        return new Vector2d(
                current.position.x + lookaheadDistance * Math.cos(target.heading.toDouble()),
                current.position.y + lookaheadDistance * Math.sin(target.heading.toDouble())
        );
    }
}
