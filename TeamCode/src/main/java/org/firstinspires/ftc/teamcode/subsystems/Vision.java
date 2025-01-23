package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision extends Mechanism {

    private Limelight3A camera;
    private LLResult result;

    @Override
    public void init(HardwareMap hwMap) {
        camera = hwMap.get(Limelight3A.class, "cam");
        camera.setPollRateHz(100);
        camera.start();
    }
}
