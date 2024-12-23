package org.firstinspires.ftc.teamcode.subsystems.v2;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.ServoState;
import org.firstinspires.ftc.teamcode.util.StateDrivenServo;

public class Wrist extends Mechanism {

    public StateDrivenServo rotator;
    public StateDrivenServo flexor;

    ServoState FLX_UP = new ServoState(0);
    ServoState FLX_NEUTRAL = new ServoState(.5);
    ServoState FLX_DOWN = new ServoState(1);

    ServoState ROT_LEFT = new ServoState(0);
    ServoState ROT_CENTER = new ServoState(.5);
    ServoState ROT_RIGHT = new ServoState(1);


    @Override
    public void init(HardwareMap hwMap) {
        flexor = new StateDrivenServo(new ServoState[]{FLX_UP, FLX_NEUTRAL, FLX_DOWN}, FLX_NEUTRAL, ConfigurationInfo.flexor.getDeviceName());
        rotator = new StateDrivenServo(new ServoState[]{ROT_LEFT, ROT_CENTER, ROT_RIGHT}, ROT_CENTER, ConfigurationInfo.rotator.getDeviceName());
        flexor.init(hwMap);
        rotator.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad) {
        rotator.loop(aimpad);
        flexor.loop(aimpad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        flexor.telemetry(telemetry);
        rotator.telemetry(telemetry);
    }
}
