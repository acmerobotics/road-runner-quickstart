package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot extends Mechanism {

    public Drivebase drivebase;
    public Hubs hubs;
    //TODO Add other subsystems here

    @Override
    public void init(HardwareMap hwMap) {
        drivebase = new Drivebase();
        hubs = new Hubs();

        drivebase.init(hwMap);
        hubs.init(hwMap);
        //TODO Initialize other subsystems here
    }

    @Override
    public void loop(AIMPad gamepad1, AIMPad gamepad2) {
        drivebase.drive(gamepad1);
        //TODO Loop other subsystems here
    }
}
