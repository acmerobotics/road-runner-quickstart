package org.firstinspires.ftc.teamcode.huskyteers.vision;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class PixelDetection {
    public TfodProcessor tfodProcessor;

    public PixelDetection() {
        tfodProcessor = new TfodProcessor.Builder().build();
    }
}

