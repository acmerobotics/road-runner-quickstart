package org.firstinspires.ftc.teamcode.subsystems.v1;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.util.ServoState;
import org.firstinspires.ftc.teamcode.util.StateDrivenServo;

public class Outtake extends Mechanism {

    StateDrivenServo leftArmHinge;
    StateDrivenServo rightArmHinge;
    StateDrivenServo bucketHinge;

    ServoState ARM_IN = new ServoState(0.05);
    ServoState ARM_OUT = new ServoState(1);

    ServoState BUCKET_IN = new ServoState(1);
    ServoState BUCKET_OUT = new ServoState(0);

    /**
     * initializes hardware
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        leftArmHinge = new StateDrivenServo(new ServoState[]{ARM_IN, ARM_OUT}, ARM_IN, ConfigurationInfo.leftArmHinge.getDeviceName(), Servo.Direction.REVERSE);
        rightArmHinge = new StateDrivenServo(new ServoState[]{ARM_IN, ARM_OUT}, ARM_IN, ConfigurationInfo.rightArmHinge.getDeviceName());
        bucketHinge = new StateDrivenServo(new ServoState[]{BUCKET_IN, BUCKET_OUT}, BUCKET_IN, ConfigurationInfo.bucketHinge.getDeviceName());

        leftArmHinge.init(hwMap);
        rightArmHinge.init(hwMap);
        bucketHinge.init(hwMap);
    }

    /**
     * creates loop of arm states and bucket states to be called
     * @param aimpad references AIMPad in slot one
     */
    @Override
    public void loop(AIMPad aimpad) {
        leftArmHinge.loop(aimpad);
        rightArmHinge.loop(aimpad);
        bucketHinge.loop(aimpad);
    }

    public void armIn() {
        leftArmHinge.setActiveTargetState(ARM_IN);
        rightArmHinge.setActiveTargetState(ARM_IN);
    }

    public void armOut() {
        leftArmHinge.setActiveTargetState(ARM_OUT);
        rightArmHinge.setActiveTargetState(ARM_OUT);
    }

    public void armCustom(double position) {
        leftArmHinge.setActiveStateCustom(position);
        rightArmHinge.setActiveStateCustom(position);
    }

    public void bucketIn() {
        bucketHinge.setActiveTargetState(BUCKET_IN);
    }

    public void bucketOut() {
        bucketHinge.setActiveTargetState(BUCKET_OUT);
    }

    public void bucketCustom(double position) {
        bucketHinge.setActiveStateCustom(position);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
    }

    public void systemsCheck(AIMPad aimpad, Telemetry telemetry) {
        if (aimpad.isDPadUpPressed()) {
            armIn();
        } else if (aimpad.isDPadDownPressed()) {
            armOut();
        } else if (aimpad.isDPadLeftPressed()) {
            bucketIn();
        } else if (aimpad.isDPadRightPressed()) {
            bucketOut();
        }
        loop(aimpad);
    }
}

