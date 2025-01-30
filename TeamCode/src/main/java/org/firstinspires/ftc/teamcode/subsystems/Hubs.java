package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Hubs extends Mechanism {

    List<LynxModule> allHubs;

    @Override
    public void init(HardwareMap hwMap) {
        allHubs = hwMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop(AIMPad aimpad) {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
