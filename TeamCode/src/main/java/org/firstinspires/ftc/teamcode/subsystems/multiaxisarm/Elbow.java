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

    ServoState DOWN = new ServoState(.92);
    ServoState HANG = new ServoState(.7);
    ServoState IN_LINE = new ServoState(.40);
    ServoState SCORE = new ServoState(.32);
    ServoState SCORE_SPECIMEN = new ServoState(.485);
    ServoState UP = new ServoState(0);

    @Override
    public void init(HardwareMap hwMap) {
        leftElbow = new StateDrivenServo(new ServoState[]{DOWN, HANG, UP, SCORE, SCORE_SPECIMEN, IN_LINE}, UP, ConfigurationInfo.leftElbow.getDeviceName());
        rightElbow = new StateDrivenServo(new ServoState[]{DOWN, HANG, UP, SCORE, SCORE_SPECIMEN, IN_LINE}, UP, ConfigurationInfo.rightElbow.getDeviceName(), Servo.Direction.REVERSE);

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

    public void score() {
        leftElbow.setActiveTargetState(SCORE);
        rightElbow.setActiveTargetState(SCORE);
    }

    public void scoreSpecimen() {
        leftElbow.setActiveTargetState(SCORE_SPECIMEN);
        rightElbow.setActiveTargetState(SCORE_SPECIMEN);
    }

    public void hang() {
        leftElbow.setActiveTargetState(HANG);
        rightElbow.setActiveTargetState(HANG);
    }

    public void toggleSpecimen() {
        if (leftElbow.getActiveTargetState() == SCORE) {
            scoreSpecimen();
        } else {
            score();
        }
    }

    public void custom(double position) {
        leftElbow.setActiveStateCustom(position);
        rightElbow.setActiveStateCustom(position);
    }
}
