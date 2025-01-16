package org.firstinspires.ftc.teamcode.subsystems.multiaxisarm;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.ServoState;
import org.firstinspires.ftc.teamcode.util.StateDrivenServo;

public class Elbow extends Mechanism {
    public StateDrivenServo elbow;

    ServoState UP = new ServoState(.7);
    ServoState AVOID = new ServoState(.5);
    ServoState DOWN = new ServoState(.1);

    @Override
    public void init(HardwareMap hwMap) {
        elbow = new StateDrivenServo(new ServoState[]{UP, DOWN, AVOID}, DOWN, ConfigurationInfo.elbow.getDeviceName(), Servo.Direction.REVERSE);
        elbow.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad) {
        elbow.loop(aimpad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        elbow.telemetry(telemetry);
    }

    public void up() {
        elbow.setActiveTargetState(UP);
    }

    public void down() {
        elbow.setActiveTargetState(DOWN);
    }

    public void avoid() {
        elbow.setActiveTargetState(AVOID);
    }

    public void custom(double position) {
        elbow.setActiveStateCustom(position);
    }
}
