package org.firstinspires.ftc.teamcode.huskyteers.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyVision {

    private SideAprilTagDetection satd = null;
    public BackdropAprilTagDetection backdropAprilTagDetection = null;

    public HuskyVision(HardwareMap hwMap) {
        backdropAprilTagDetection = new BackdropAprilTagDetection(hwMap);
    }
}
