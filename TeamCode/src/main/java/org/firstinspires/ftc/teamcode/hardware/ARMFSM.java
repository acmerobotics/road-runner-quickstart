package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;

public class ARMFSM extends Mechanism{
    private ScoringArm arm = new ScoringArm();
    private Telemetry telemetry;
    public enum states {
        down,
        up,
        dump
    };
    public states
    @Override
    public void init(HardwareMap hwMap) {
        arm.init(hwMap);
    }
    public void init(HardwareMap hwMap, Telemetry tele) {
        arm.init(hwMap);
        telemetry = tele;
    }

}
