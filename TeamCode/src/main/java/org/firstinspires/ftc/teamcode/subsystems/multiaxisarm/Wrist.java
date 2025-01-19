package org.firstinspires.ftc.teamcode.subsystems.multiaxisarm;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.ServoState;
import org.firstinspires.ftc.teamcode.util.StateDrivenServo;

public class Wrist extends Mechanism {

    public StateDrivenServo rotator;
    public StateDrivenServo flexor;

    ServoState FLX_UP = new ServoState(.05);
    ServoState FLX_NEUTRAL = new ServoState(.5);
    ServoState FLX_DOWN = new ServoState(1);

    ServoState ROT_LEFT = new ServoState(0.05);
    ServoState ROT_CENTER = new ServoState(.47);
    ServoState ROT_RIGHT = new ServoState(.89);


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

    public void flexUp() {
        flexor.setActiveTargetState(FLX_UP);
    }

    public void flexNeutral() {
        flexor.setActiveTargetState(FLX_NEUTRAL);
    }

    public void flexDown() {
        flexor.setActiveTargetState(FLX_DOWN);
    }

    public void flexCustom(double position) {
        flexor.setActiveStateCustom(position);
    }

    public void rotateLeft() {
        rotator.setActiveTargetState(ROT_LEFT);
    }

    public void rotateCenter() {
        rotator.setActiveTargetState(ROT_CENTER);
    }

    public void rotateRight() {
        rotator.setActiveTargetState(ROT_RIGHT);
    }

    public void rotateCustom(double position) {
        rotator.setActiveStateCustom(position);
    }
}
