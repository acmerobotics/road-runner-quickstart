package org.firstinspires.ftc.teamcode.subsystems;

import android.content.res.Configuration;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

public class Intake extends Mechanism {

    CRServo bristles;
    Servo leftHinge;
    Servo rightHinge;
    final double DOWN_HINGE_POSITION = -0.5;
    final double NEUTRAL_HINGE_POSITION = 0;
    final double UP_HINGE_POSITION = 0.5;

    enum HingeState {
        DOWN, NEUTRAL, UP, CUSTOM
    }

    HingeState activeHingeState = HingeState.UP;

    double hingeTargetPosition = UP_HINGE_POSITION;


    @Override
    public void init(HardwareMap hwMap) {
        bristles = hwMap.get(CRServo.class, ConfigurationInfo.bristles.getDeviceName());
        leftHinge = hwMap.get(Servo.class, ConfigurationInfo.leftHinge.getDeviceName());
        rightHinge = hwMap.get(Servo.class, ConfigurationInfo.rightHinge.getDeviceName());
    }

    @Override
    public void loop(AIMPad aimpad) {
        switch(activeHingeState) {
            case DOWN:
                downState();
                break;
            case NEUTRAL:
                neutralState();
                break;
            case UP:
                upState();
                break;
            case CUSTOM:
                break;
        }
        hingeToPosition(hingeTargetPosition);
    }

    public void setActiveHingeState(HingeState activeHingeState) {
        this.activeHingeState = activeHingeState;
    }

    public void setHingeStateCustom(double position) {
        setActiveHingeState(HingeState.CUSTOM);
        hingeTargetPosition = position;
    }

    public void downState() {
        hingeTargetPosition = DOWN_HINGE_POSITION;
    }

    public void upState() {
        hingeTargetPosition = UP_HINGE_POSITION;
    }

    public void neutralState() {
        hingeTargetPosition = NEUTRAL_HINGE_POSITION;
    }



    public void bristlesIn() {bristles.setPower(1);}

    public void bristlesOut() {bristles.setPower(-1);}

    public void bristlesOff(){
        bristles.setPower(0);
    }

    public void bristlesAtPower(double bristlesPower) {
        bristles.setPower(bristlesPower);
    }


    public void hingeNeutral() {
        hingeToPosition(NEUTRAL_HINGE_POSITION);
    }

    public void hingeUp() {
        hingeToPosition(UP_HINGE_POSITION);
    }

    public void hingeDown() {
        hingeToPosition(DOWN_HINGE_POSITION);
    }

    public void hingeToPosition(double hingePosition) {
        double clampedHingePosition = Math.max(DOWN_HINGE_POSITION, Math.min(hingePosition, UP_HINGE_POSITION));
        leftHinge.setPosition(clampedHingePosition);
        rightHinge.setPosition(clampedHingePosition);
    }

    public void systemsCheck(AIMPad aimpad) {
        loop(aimpad);
        if (aimpad.isAPressed()) {
            setActiveHingeState(HingeState.UP);
        } else if (aimpad.isBPressed()) {
            setActiveHingeState(HingeState.DOWN);
        } else if (aimpad.isXPressed()) {
            setActiveHingeState(HingeState.NEUTRAL);
        } else if (aimpad.isLeftStickMovementEngaged()) {
            setHingeStateCustom(aimpad.getLeftStickX());
        }
    }

}
