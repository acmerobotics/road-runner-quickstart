
package org.firstinspires.ftc.teamcode.huskyteers.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class BackdropAprilTagDetection {
    // Backdrop April Tag IDs
    // Blue Left: 1, Blue Center: 2, Blue Right: 3
    // Red Left: 4, Red Center 5, Red Right: 6

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final HardwareMap hwMap;

    public BackdropAprilTagDetection(HardwareMap hwMap){
        this.hwMap = hwMap;

        initAprilTag();
    }

    public AprilTagDetection closestAprilTag() {
        // Step through the list of detected tags and look for closest one.
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if(currentDetections.size() == 0)
            return null;


        AprilTagDetection closestTag = currentDetections.get(0);

        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null)){
                if(detection.ftcPose.range < closestTag.ftcPose.range){
                    closestTag = detection;
                }
            }
        }

        return closestTag;
    }

    public AprilTagDetection getAprilTagById(int id) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection detectedTag = null;

        for(AprilTagDetection detection : currentDetections){
            if(detection.metadata != null) {
                if (detection.id == id) {
                    detectedTag = detection;
                    break;
                }
            }
        }
        return detectedTag;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280,720))
                .addProcessor(aprilTag)
                .build();

        // Manually set the camera gain and exposure.
        // ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        // exposureControl.setExposure((long)6, TimeUnit.MILLISECONDS);
        // GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        // gainControl.setGain(250);
    }
}

