package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class OctoQuadLocalizer implements Localizer {
    public static class Params {

        /*
        Set these to the numbers of the ports that the X and Y odometry are plugged into on the OctoQuad.
         */
        public int odometryPortX = 1;

        public int odometryPortY = 2;

        /*
        The OctoQuad IMU needs to be tuned before use to ensure the output heading is accurate.
        Run AngularScalarTuner and follow the instructions to get this value.
         */
        public double angularScalar = 1.0415;

        /*
        Set the odometry pod positions relative to the center of the robot.
        The X pod offset refers to how far sideways from the center the X (forward) odometry pod is.
        Left of the center is a positive number, right of the center is a negative number.
        The Y pod offset refers to how far forwards from the center the Y (strafe) odometry pod is:
        forward of the center is a positive number, backwards is a negative number.
         */
        // These are tuned for 3110-0002-0001 Product Insight #1
        public double xOffset = -5.24373777; // inches
        public double yOffset = -3.412719295440588; // inches
        // todo: match pinpointlocalizer

        /*
        Set the encoder resolution of your odometry pods in ticks per millimeter.

        This is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; for example, the value for the Gobilda 4 Bar Odometry Pod is 19.89436789.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)
         */
        public double encoderResolution = 19.89436789; // ticks / mm
        // todo match pinpoint localizer

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        public OctoQuadFWv3.EncoderDirection xDirection = OctoQuadFWv3.EncoderDirection.FORWARD;
        public OctoQuadFWv3.EncoderDirection yDirection = OctoQuadFWv3.EncoderDirection.REVERSE;

    }

    public static Params PARAMS = new Params();

    public final OctoQuadFWv3 octoquad;
    private Pose2d currentPose;

    public OctoQuadLocalizer(HardwareMap hardwareMap, Pose2d initialPose) throws InterruptedException {
        FlightRecorder.write("OCTOQUAD_PARAMS", PARAMS);
        // TODO: make sure your config has an OctoQuad device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        octoquad = hardwareMap.get(OctoQuadFWv3.class, "octoquad");
        currentPose = initialPose;

        octoquad.setLocalizerPortX(PARAMS.odometryPortX);
        octoquad.setLocalizerPortY(PARAMS.odometryPortY);

        octoquad.setSingleEncoderDirection(PARAMS.odometryPortX, PARAMS.xDirection);
        octoquad.setSingleEncoderDirection(PARAMS.odometryPortY, PARAMS.yDirection);

        octoquad.setLocalizerTcpOffsetMM_X((float) DistanceUnit.MM.fromInches(PARAMS.xOffset));
        octoquad.setLocalizerTcpOffsetMM_Y((float) DistanceUnit.MM.fromInches(PARAMS.yOffset));


        octoquad.setLocalizerCountsPerMM_X((float) PARAMS.encoderResolution);
        octoquad.setLocalizerCountsPerMM_Y((float) PARAMS.encoderResolution);

        octoquad.setLocalizerImuHeadingScalar((float) PARAMS.angularScalar);

        octoquad.setLocalizerVelocityIntervalMS(25);


        /*
        Reset the localization and calibrate the IMU.
         */
        octoquad.resetLocalizerAndCalibrateIMU();
        ElapsedTime timeout = new ElapsedTime();
        OctoQuadFWv3.LocalizerStatus currentStatus = octoquad.getLocalizerStatus();
        while (currentStatus != OctoQuadFWv3.LocalizerStatus.RUNNING) {
            Thread.sleep(10);
            if (timeout.seconds() > 3) {
                Log.println(
                        Log.WARN,
                        "OctoQuadLocalizer",
                        "init: Calibration timeout reached, OQ still not ready. OctoQuad reports " + currentStatus
                );
                RobotLog.addGlobalWarningMessage(
                        "OctoQuad still not ready, timeout reached. OctoQuad reports " + currentStatus +
                                " Continuing anyway, good luck!"
                );
                break;
            }
            currentStatus = octoquad.getLocalizerStatus();
        }

        octoquad.setLocalizerPose(
                (int) DistanceUnit.MM.fromInches(currentPose.position.x),
                (int) DistanceUnit.MM.fromInches(currentPose.position.y),
                (float) currentPose.heading.toDouble()
        );
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
        octoquad.setLocalizerPose(
                (int) DistanceUnit.MM.fromInches(currentPose.position.x),
                (int) DistanceUnit.MM.fromInches(currentPose.position.y),
                (float) currentPose.heading.toDouble()
        );
    }

    @Override
    public PoseVelocity2d update() {
        OctoQuadFWv3.LocalizerDataBlock localizer = new OctoQuadFWv3.LocalizerDataBlock();
        octoquad.readLocalizerData(localizer);
        if (localizer.localizerStatus != OctoQuadFWv3.LocalizerStatus.RUNNING || !localizer.crcOk
        ) {
            Log.println(
                    Log.WARN,
                    "OctoQuadLocalizer",
                    "update: Bad data recieved from OctoQuad." +
                            " Localizer status: " + localizer.localizerStatus +
                            " CRC OK: " + localizer.crcOk
            );
            // throw away bad data (don't change pose)
            // have to return something for velocity; hopefully this won't cause a sudden jump? need to test
            return new PoseVelocity2d(new Vector2d(0,0),0);
        }
        currentPose = new Pose2d(
                DistanceUnit.INCH.fromMm(localizer.posX_mm),
                DistanceUnit.INCH.fromMm(localizer.posY_mm),
                localizer.heading_rad);

        Vector2d fieldVel = new Vector2d(DistanceUnit.INCH.fromMm(localizer.velX_mmS),
                DistanceUnit.INCH.fromMm(localizer.velY_mmS));
        Vector2d robotVel = fieldVel.times(localizer.velHeading_radS);
        return new PoseVelocity2d(robotVel, localizer.velHeading_radS);
    }
}
