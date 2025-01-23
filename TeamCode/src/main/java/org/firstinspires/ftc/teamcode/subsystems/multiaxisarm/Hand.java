package org.firstinspires.ftc.teamcode.subsystems.multiaxisarm;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.aimrobotics.aimlib.subsystems.sds.StateDrivenServo;
import com.aimrobotics.aimlib.subsystems.sds.ServoState;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Hand extends Mechanism {
    public StateDrivenServo hand;

    ServoState CLOSED = new ServoState(0.44);
    ServoState OPEN = new ServoState(1);

    @Override
    public void init(HardwareMap hwMap) {
        hand = new StateDrivenServo(new ServoState[]{CLOSED, OPEN}, OPEN, ConfigurationInfo.hand.getDeviceName());
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
    }

    public void close() {
        hand.setActiveTargetState(CLOSED);
    }

    public void custom(double position) {
        hand.setActiveStateCustom(position);
    }
}
