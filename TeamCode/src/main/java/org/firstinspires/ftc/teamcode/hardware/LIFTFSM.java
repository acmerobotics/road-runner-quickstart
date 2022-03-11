package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LIFTFSM extends Mechanism{
    private Telemetry telemetry;

    private Lift lift = new Lift();
    public enum states {
        low,
        mid,
        high,
    };
    public states scoreState;
    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        lift.lower();
        scoreState = states.low;
    }
    public void init(HardwareMap hwMap, Telemetry tele) {
        this.init(hwMap);
        telemetry = tele;
    }

    public void loop() {
        switch(scoreState) {
            case low:
                lift.retracting(true);
                lift.lower();
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
        scoreState = states.high;
    }
    public void goMid() {
        scoreState = states.mid;
    }
    public void goLow() {
        scoreState = states.low;
    }

}
