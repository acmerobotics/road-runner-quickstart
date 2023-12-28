package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Camera extends Mechanism {
    private static VisionPortal visionPortal; // Used to manage the video source.
    private static AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process
    private static AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag
    boolean targetFound = false; // Is the target tag currently detected?
    static final int exposureMS = 4; // ms exposure
    static final int gain = 250; // gain

    // Tag IDs
    public static final int BLUE_LEFT_ID = 1;
    public static final int BLUE_CENTER_ID = 2;
    public static final int BLUE_RIGHT_ID = 3;
    public static final int RED_LEFT_ID = 4;
    public static final int RED_CENTER_ID = 5;
    public static final int RED_RIGHT_ID = 6;

    private static TfodProcessor tfod; // Used for managing the TensorFlow Object detection process

    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite"; // Name of the model asset

    private static final String[] LABELS = { // Labels for the model
            "Pixel"
            // "Blue Element",
            // "Red Element"
    };

    private static final int LEFT_MAX_THRESHOLD = 213;
    private static final int CENTER_MAX_THRESHOLD = 427;

    /**
     * Initializes the camera and vision processors.
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, ConfigInfo.camera.getDeviceName()))
                .addProcessor(aprilTag)
                .addProcessor(tfod)
                .build();
        aprilTag.setDecimation(3);
        setManualExposure(exposureMS, gain);
        tfod.setMinResultConfidence(0.75f);
    }

    /**
     * Updates the telemetry with the current camera data.
     * @param telemetry references local telemetry
     */
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetryTag(telemetry);
        telemetryTfod(telemetry);
    }

    /**
     * Updates the camera exposure and gain settings
     * @param exposureMS the exposure in milliseconds
     * @param gain the gain
     */
    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) { // Check if the camera is streaming
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    /**
     * Checks to see if the desired tag is detected and sets the targetFound variable and metadata accordingly
     * @param desiredTagID the ID of the desired tag
     */
    public void checkAndSetDesiredTag(int desiredTagID) {
        List<AprilTagDetection> currentDetections = getDetections();
        boolean detectionFound = false; // Add this variable

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                // Check to see if we want to track towards this tag.
                if ((desiredTagID < 0) || (detection.id == desiredTagID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    detectionFound = true; // Set detectionFound to true
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    // You can remove this line, as it's not necessary.
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                // You can remove this line, as it's not necessary.
            }
        }

        // After the loop, set targetFound based on detectionFound
        if (!detectionFound) {
            targetFound = false;
        }
    }

    /**
     * Retruns the desired tag's pose data
     * @return pose data of the desired tag
     */
    public double[] getDesiredTagPoseData() {
        if (targetFound) {
            return new double[] {desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw};
        } else {
            return null;
        }
    }

    /**
     * Returns the camera state
     * @return the camera state
     */
    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    /**
     * Returns the list of detections
     * @return the list of detections
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Telemetry on desired tag pose data based on AprilTag detection
     * @param telemetry references local telemetry
     */
    private void telemetryTag(Telemetry telemetry) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) { // Check if the camera is streaming
            if (targetFound) {
                telemetry.addData("Tag ID ", desiredTag.id);
                telemetry.addData("Tag Name ", desiredTag.metadata.name);
                telemetry.addData("Tag X ", desiredTag.ftcPose.x);
                telemetry.addData("Tag Y ", desiredTag.ftcPose.y);
                telemetry.addData("Tag Z ", desiredTag.ftcPose.z);
                telemetry.addData("Tag Pitch ", desiredTag.ftcPose.pitch);
                telemetry.addData("Tag Roll ", desiredTag.ftcPose.roll);
                telemetry.addData("Tag Yaw ", desiredTag.ftcPose.yaw);
                telemetry.addData("Tag Range ", desiredTag.ftcPose.range);
                telemetry.addData("Tag Bearing ", desiredTag.ftcPose.bearing);
                telemetry.addData("Tag Elevation ", desiredTag.ftcPose.elevation);
            } else {
                telemetry.addLine("No TAG");
            }
        }
    }

    /**
     * Telemetry on TensorFlow Object Detection
     * @param telemetry references local telemetry
     */
    private void telemetryTfod(Telemetry telemetry) {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }

    /**
     * Returns the predicted position of the element based on TensorFlow Object Detection
     * @return randomization of the match
     */
    public int getTfodElementPos() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getLabel().equals(LABELS[0])) { // TODO Update for detections
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                if (x < LEFT_MAX_THRESHOLD) {
                    return 1;
                } else if (x < CENTER_MAX_THRESHOLD) {
                    return 2;
                } else {
                    return 3;
                }
            }
        }
        return 1;
    }
}
