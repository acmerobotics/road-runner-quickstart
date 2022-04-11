package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class LIFTFSM extends Mechanism{
    private Telemetry telemetry;

    public static int counter = 0;

    public Lift lift = new Lift();
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
        init(hwMap);
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
                lift.setTargetPosition(-1);
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


    private boolean estimatedEqual(double one, double two){
        return Math.abs(one - two) < 0.5;
    }

}
