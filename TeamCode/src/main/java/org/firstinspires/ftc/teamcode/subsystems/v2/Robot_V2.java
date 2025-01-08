package org.firstinspires.ftc.teamcode.subsystems.v2;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.generic.Hubs;

public class Robot_V2 extends Mechanism {

    Drivebase drivebase = new Drivebase();
    Hubs hubs = new Hubs();
    ScoringAssembly scoringAssembly = new ScoringAssembly();

    @Override
    public void init(HardwareMap hwMap) {
        drivebase.init(hwMap);
        hubs.init(hwMap);
        scoringAssembly.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad, AIMPad aimpad2) {

    }
}
