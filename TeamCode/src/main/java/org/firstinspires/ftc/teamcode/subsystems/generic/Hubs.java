package org.firstinspires.ftc.teamcode.subsystems.generic;

import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Hubs extends Mechanism {
    @Override
    public void init(HardwareMap hwMap) {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
