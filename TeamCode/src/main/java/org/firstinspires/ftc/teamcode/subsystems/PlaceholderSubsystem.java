package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import java.net.InetAddress;

public class PlaceholderSubsystem extends Mechanism {
    Limelight3A limelight;


    @Override
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
    }
}
