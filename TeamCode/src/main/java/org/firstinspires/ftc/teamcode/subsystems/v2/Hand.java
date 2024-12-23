package org.firstinspires.ftc.teamcode.subsystems.v2;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.ServoState;
import org.firstinspires.ftc.teamcode.util.StateDrivenServo;

public class Hand extends Mechanism {
    public StateDrivenServo hand;

    ServoState CLOSED = new ServoState(0);
    ServoState OPEN = new ServoState(1);

    @Override
    public void init(HardwareMap hwMap) {
        hand = new StateDrivenServo(new ServoState[]{CLOSED, OPEN}, OPEN, ConfigurationInfo.hand.getDeviceName());
    }

    @Override
    public void loop(AIMPad aimpad) {
        hand.loop(aimpad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        hand.telemetry(telemetry);
    }
}
