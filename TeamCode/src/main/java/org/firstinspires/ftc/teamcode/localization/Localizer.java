package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * Interface for localization methods.
 */
public interface Localizer {
    /**
     * Updates the position estimate.
     * @param pose the current position estimate
     * @return the updated position estimate
     */
    Pose2d updatePositionEstimate(Pose2d pose);

    /**
     * Updates the velocity estimate.
     * @return the updated velocity estimate
     */
    PoseVelocity2d updateVelocityEstimate();
}
