package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.MultiAxisArm;

public class ScoringAssembly extends Mechanism {

    public MultiAxisArm multiAxisArm;
    public Pivot pivot;
    public Slides slides;

    @Override
    public void init(HardwareMap hwMap) {
        multiAxisArm = new MultiAxisArm();
        multiAxisArm.init(hwMap);
        pivot = new Pivot();
        pivot.init(hwMap);
        slides = new Slides();
        slides.init(hwMap);
    }

    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        pivot.loop(aimpad);
        slides.loop(aimpad);
        multiAxisArm.loop(aimpad);
    }

}
