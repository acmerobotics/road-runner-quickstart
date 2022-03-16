package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LIFTFSM extends Mechanism{
    private Telemetry telemetry;

    private Lift lift = new Lift();
    public enum states {
        low,
        mid,
        high,
    };
    public states liftState;
    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        lift.lower();
        liftState = states.low;
    }
    public void init(HardwareMap hwMap, Telemetry tele) {
        this.init(hwMap);
        telemetry = tele;
    }

    public void loop() {
        switch(liftState) {
            case low:
                if(lift.getCurrentPosition() >= lift.midPos + 2) {
                    lift.retracting(true);
                }else {
                    lift.retracting(false);
                }
                lift.setTargetPosition(lift.minPos);
                break;
            case mid:
                lift.retracting(false);
                lift.raiseMid();
                break;
            case high:
                lift.raiseHigh();
                break;
        }
        lift.update();
    }
    public void goHigh() {
        liftState = states.high;
    }
    public void goMid() {
        liftState = states.mid;
    }
    public void goLow() {
        liftState = states.low;
    }

}
