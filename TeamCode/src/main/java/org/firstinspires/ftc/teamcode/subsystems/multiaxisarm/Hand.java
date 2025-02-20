package org.firstinspires.ftc.teamcode.subsystems.multiaxisarm;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.aimrobotics.aimlib.subsystems.sds.StateDrivenServo;
import com.aimrobotics.aimlib.subsystems.sds.ServoState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Hand extends Mechanism {
    public StateDrivenServo hand;

    public enum HandState {
        OPEN,
        CLOSED
    }

    public HandState activeHandState = HandState.OPEN;

    ServoState CLOSED = new ServoState(0.38);
    ServoState OPEN = new ServoState(.82);

    @Override
    public void init(HardwareMap hwMap) {
        hand = new StateDrivenServo(new ServoState[]{CLOSED, OPEN}, OPEN, ConfigurationInfo.hand.getDeviceName());
        activeHandState = HandState.OPEN;
        hand.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad) {
        hand.loop(aimpad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        hand.telemetry(telemetry);
    }

    public void open() {
        hand.setActiveTargetState(OPEN);
        activeHandState = HandState.OPEN;
    }

    public void close() {
        hand.setActiveTargetState(CLOSED);
        activeHandState = HandState.CLOSED;
    }

    public void custom(double position) {
        hand.setActiveStateCustom(position);
    }

    public void toggle() {
        if (hand.getActiveTargetState() == OPEN) {
            close();
        } else {
            open();
        }
    }

}
