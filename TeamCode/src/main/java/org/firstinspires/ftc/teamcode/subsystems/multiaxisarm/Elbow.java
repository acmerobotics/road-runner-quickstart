package org.firstinspires.ftc.teamcode.subsystems.multiaxisarm;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.aimrobotics.aimlib.subsystems.sds.StateDrivenServo;
import com.aimrobotics.aimlib.subsystems.sds.ServoState;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class Elbow extends Mechanism {
    public StateDrivenServo leftElbow;
    public StateDrivenServo rightElbow;

    ServoState UP = new ServoState(.7);
    ServoState MIDDLE = new ServoState(.5);
    ServoState DOWN = new ServoState(.2);

    @Override
    public void init(HardwareMap hwMap) {
        leftElbow = new StateDrivenServo(new ServoState[]{UP, DOWN, MIDDLE}, DOWN, ConfigurationInfo.leftElbow.getDeviceName());
        rightElbow = new StateDrivenServo(new ServoState[]{UP, DOWN, MIDDLE}, DOWN, ConfigurationInfo.rightElbow.getDeviceName(), Servo.Direction.REVERSE);

        leftElbow.init(hwMap);
        rightElbow.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad) {
        leftElbow.loop(aimpad);
        rightElbow.loop(aimpad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        leftElbow.telemetry(telemetry);
        rightElbow.telemetry(telemetry);
    }

    public void up() {
        leftElbow.setActiveTargetState(UP);
        rightElbow.setActiveTargetState(UP);
    }

    public void down() {
        leftElbow.setActiveTargetState(DOWN);
        rightElbow.setActiveTargetState(DOWN);
    }

    public void middle() {
        leftElbow.setActiveTargetState(MIDDLE);
        rightElbow.setActiveTargetState(MIDDLE);
    }

    public void custom(double position) {
        leftElbow.setActiveStateCustom(position);
        rightElbow.setActiveStateCustom(position);
    }
}
