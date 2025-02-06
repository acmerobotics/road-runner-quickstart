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

    ServoState DOWN = new ServoState(1);
    ServoState IN_LINE = new ServoState(.61);
    ServoState UP = new ServoState(0);

    @Override
    public void init(HardwareMap hwMap) {
        leftElbow = new StateDrivenServo(new ServoState[]{DOWN, UP, IN_LINE}, UP, ConfigurationInfo.leftElbow.getDeviceName());
        rightElbow = new StateDrivenServo(new ServoState[]{DOWN, UP, IN_LINE}, UP, ConfigurationInfo.rightElbow.getDeviceName(), Servo.Direction.REVERSE);

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

    public void forward() {
        leftElbow.setActiveTargetState(DOWN);
        rightElbow.setActiveTargetState(DOWN);
    }

    public void backward() {
        leftElbow.setActiveTargetState(UP);
        rightElbow.setActiveTargetState(UP);
    }

    public void middle() {
        leftElbow.setActiveTargetState(IN_LINE);
        rightElbow.setActiveTargetState(IN_LINE);
    }

    public void custom(double position) {
        leftElbow.setActiveStateCustom(position);
        rightElbow.setActiveStateCustom(position);
    }
}
