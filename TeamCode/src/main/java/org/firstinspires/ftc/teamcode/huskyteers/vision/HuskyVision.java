package org.firstinspires.ftc.teamcode.huskyteers.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class HuskyVision {

    public BackdropAprilTagDetection backdropAprilTagDetection;
    public VisionPortal visionPortal;

    public HuskyVision(HardwareMap hwMap) {
        backdropAprilTagDetection = new BackdropAprilTagDetection();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(backdropAprilTagDetection.aprilTag)
                .build();
    }
}
