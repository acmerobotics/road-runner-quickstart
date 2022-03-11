package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;

public class ARMFSM extends Mechanism{
    private ScoringArm arm = new ScoringArm();
    private ElapsedTime timer = new ElapsedTime();
    private Telemetry telemetry;
    public enum states {
        down,
        up,
        dump,
        postDump,
    };
    public states armStates;
    @Override
    public void init(HardwareMap hwMap) {
        arm.init(hwMap);
        armStates = states.down;
    }
    public void init(HardwareMap hwMap, Telemetry tele) {
        arm.init(hwMap);
        telemetry = tele;
    }
    public void loop() {
        switch(armStates) {
            case down:
                arm.goToStart();
                arm.depositReset();
                break;
            case up:
                arm.goToEnd();
                break;
            case dump:
                timer.reset();
                arm.dumpHard();
                armStates = states.postDump;
                break;
            case postDump:
                if(timer.milliseconds() >= 100) {
                    armStates = states.down;
                }
        }
    }
    public void ready() {
        armStates = states.up;
    }
    public void down() {
        armStates = states.down;
    }
    public void dump() {
        armStates = states.dump;
    }
}
