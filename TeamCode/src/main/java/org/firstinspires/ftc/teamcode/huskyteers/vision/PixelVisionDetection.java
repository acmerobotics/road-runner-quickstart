package org.firstinspires.ftc.teamcode.huskyteers.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class PixelVisionDetection {
    private VisionPortal visionPortal;
    private HardwareMap hwMap;
    private TfodProcessor tfod;

    public PixelVisionDetection(HardwareMap hwMap){
        this.hwMap = hwMap;

        init();
    }


    private void init() {
        tfod = new TfodProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tfod)
                .build();
    }

}
