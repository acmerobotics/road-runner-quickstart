package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagTest {
    private static boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //Objects
    private HardwareMap hardwareMap = null;
    //public Telemetry telemetry = new TelemetryImpl((OpMode) this);
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private VisionPortal visionPortal;               // Used to manage the video source.
    private MecanumDrive driveMC = null;
    private String webcamName;
    public AprilTagTest(MecanumDrive driveMC, HardwareMap hdwmap, int tagId, String tagCamName){
        this.driveMC = driveMC;
        USE_WEBCAM = true;
        this.DESIRED_TAG_ID = tagId;
        this.hardwareMap = hdwmap;
        this.webcamName = tagCamName;
    }

    public void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }
        Logging.log("finished init");
    }

    private void detectTag() {
        targetFound = false;
        desiredTag  = null;
        Logging.log("Starting detection");
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    Logging.log("Tag found. Tag ID: %d", detection.id);
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    //telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    Logging.log("Tag not found. Detected tag ID: %d. Desired ID: %d", detection.id, DESIRED_TAG_ID);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                Logging.log("Unknown Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound) {
            Logging.log("Found ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            Logging.log("Range %5.1f inches", desiredTag.ftcPose.range);
            Logging.log("Bearing %3.0f degrees", desiredTag.ftcPose.bearing);
            Logging.log("Yaw %3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            //telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }
    }

    private boolean driveToTag() {

        //for driving
        double drive;           // Desired forward power/speed (-1 to +1)
        double strafe;          // Desired strafe power/speed (-1 to +1)
        double turn;            // Desired turning power/speed (-1 to +1)
        double SPEED_GAIN = 0.02;   //0.02;     //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        double STRAFE_GAIN = 0.015;  //0.015;    //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        double TURN_GAIN = 0.01;    //0.01;     //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        if (targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            Logging.log("Auto Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else {
            drive = 0;
            turn = 0;
            strafe = 0;
        }

        Logging.log("Driving to target");
        boolean reachTarget = moveRobot(drive, strafe, turn);
        Logging.log("Drove to target");
        // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
        //drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
        //strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
        //turn   = -gamepad1.right_stick_x / 2.0;  // Reduce turn rate to 50%.
        //telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        //telemetry.update();
        // Apply desired axes motions to the drivetrain.

        return reachTarget;
    }
    private boolean moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        Logging.log(" Max power setting = % 2.2f", max);
        if (0 < max && max < 0.1) {
            return true;
        }

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        driveMC.leftFront.setPower(leftFrontPower);
        driveMC.rightFront.setPower(rightFrontPower);
        driveMC.leftBack.setPower(leftBackPower);
        driveMC.rightBack.setPower(rightBackPower);
        return false;
    }

    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Logging.log("Camera is Waiting");
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            Logging.log("Camera is Ready");
        }

        // Set camera controls unless we are stopping.
        //if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void autoDriveToAprilTag() {
        boolean reachedTarget = false;
        int detectCount = 0;

        // exit while loop when reached target or cannot found target in 500 ms.
        while ((!reachedTarget) && (detectCount < 99)) {
            detectTag();
            if (!targetFound) {
                for (detectCount = 0; detectCount < 100; detectCount++) {
                    detectTag();
                    sleep(5);
                    if (targetFound) {
                        break;
                    }
                }
            }
            reachedTarget = driveToTag();

            Logging.log("April Tag found? %s ", targetFound ? "Yes" : "No");
            Logging.log("Reached Tag? %s ", reachedTarget ? "Yes" : "No");

            driveMC.updatePoseEstimate();

            Logging.log("Drive Heading = %2.2f", Math.toDegrees(driveMC.pose.heading.log()));
            Logging.log("Drive position x = %2.2f, y = %2.2f", driveMC.pose.position.x, driveMC.pose.position.y);
        }
    }
}