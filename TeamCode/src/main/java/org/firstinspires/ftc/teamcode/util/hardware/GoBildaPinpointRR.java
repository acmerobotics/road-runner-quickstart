/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package com.acmerobotics.roadrunner.ftc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;


@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer Roadrunner Driver",
        xmlTag = "goBILDAPinpointRR",
        description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)

public class GoBildaPinpointRR extends GoBildaPinpoint {


    public float currentTicksPerMM = 0f;

    public static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
    public static final float goBILDA_4_BAR_POD = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod


    public GoBildaPinpointRR(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }


    /**
     * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.<br><br>
     *
     * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
     */
    public void setEncoderResolution(GoBildaOdometryPods pods) {
        super.setEncoderResolution(pods);
        if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            currentTicksPerMM = goBILDA_SWINGARM_POD;
        }
        if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD) {
            currentTicksPerMM = goBILDA_4_BAR_POD;
        }
    }

    /**
     * Sets the encoder resolution in ticks per mm of the odometry pods. <br>
     * You can find this number by dividing the counts-per-revolution of your encoder by the circumference of the wheel.
     *
     * @param ticks_per_mm should be somewhere between 10 ticks/mm and 100 ticks/mm a goBILDA Swingarm pod is ~13.26291192
     */
    public void setEncoderResolution(double ticks_per_mm) {
        super.setEncoderResolution(ticks_per_mm);
        currentTicksPerMM = (float) ticks_per_mm;
    }


    /**
     * Added by j5155 (comments from Gobilda)
     * Send a position that the Pinpoint should use to track your robot relative to. You can use this to
     * update the estimated position of your robot with new external sensor data, or to run a robot
     * in field coordinates. <br><br>
     * This overrides the current position. <br><br>
     * <strong>Using this feature to track your robot's position in field coordinates:</strong> <br>
     * When you start your code, send a Pose2D that describes the starting position on the field of your robot. <br>
     * Say you're on the red alliance, your robot is against the wall and closer to the audience side,
     * and the front of your robot is pointing towards the center of the field.
     * You can send a setPosition with something like -600mm x, -1200mm Y, and 90 degrees. The pinpoint would then always
     * keep track of how far away from the center of the field you are. <br><br>
     * <strong>Using this feature to update your position with additional sensors: </strong><br>
     * Some robots have a secondary way to locate their robot on the field. This is commonly
     * Apriltag localization in FTC, but it can also be something like a distance sensor.
     * Often these external sensors are absolute (meaning they measure something about the field)
     * so their data is very accurate. But they can be slower to read, or you may need to be in a very specific
     * position on the field to use them. In that case, spend most of your time relying on the Pinpoint
     * to determine your location. Then when you pull a new position from your secondary sensor,
     * send a setPosition command with the new position. The Pinpoint will then track your movement
     * relative to that new, more accurate position.
     *
     * @param pos a Pose2D describing the robot's new position.
     */
    public Pose2d setPosition(Pose2d pos) {
        setPosition(new Pose2D(DistanceUnit.INCH, pos.position.x, pos.position.y, AngleUnit.RADIANS, pos.heading.toDouble()));
        return pos;
    }

    /**
     * Added by j5155
     *
     * @return a Roadrunner Pose2D containing the estimated position of the robot
     */
    public Pose2d getPositionRR() {
        Pose2D ftcPose = getPosition();
        return new Pose2d(
                ftcPose.getX(DistanceUnit.INCH),
                ftcPose.getY(DistanceUnit.INCH),
                ftcPose.getHeading(AngleUnit.RADIANS));
    }


    /**
     * Added by j5155
     *
     * @return a Roadrunner PoseVelocity2D containing the estimated velocity of the robot, velocity is unit per second
     */
    public PoseVelocity2d getVelocityRR() {
        Pose2D ftcVelocity = getVelocity();
        return new PoseVelocity2d(
                new Vector2d(
                        ftcVelocity.getX(DistanceUnit.INCH),
                        ftcVelocity.getY(DistanceUnit.INCH)
                ),
                ftcVelocity.getHeading(AngleUnit.RADIANS)
        );
    }

}




