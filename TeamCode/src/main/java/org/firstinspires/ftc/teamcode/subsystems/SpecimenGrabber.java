package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

import java.util.Map;

public class SpecimenGrabber extends Mechanism {

    Servo specimenGrabber;

    public enum GrabberState {
        GRAB, RELEASE, CUSTOM
    }

    GrabberState activeGrabberState = GrabberState.RELEASE;

    double targetGrabberPosition;
    private final static double GRAB_POSITION = 0.0;
    private final static double RELEASE_POSITION =  0.21;

    @Override
    public void init(HardwareMap hwMap) {
        specimenGrabber = hwMap.get(Servo.class, ConfigurationInfo.specimenGrabber.getDeviceName());
    }

    @Override
    public void loop(AIMPad aimPad) {
        switch(activeGrabberState) {
            case GRAB:
                grab();
                break;
            case RELEASE:
                release();
                break;
            case CUSTOM:
                break;
        }
        specimenGrabber.setPosition(targetGrabberPosition);
    }

    public void grab() {
        targetGrabberPosition = GRAB_POSITION;
    }

    public void release() {
        targetGrabberPosition = RELEASE_POSITION;
    }

    public void setGrabberState(GrabberState state) {
        activeGrabberState = state;
    }

    public void setGrabberStateCustom(double position) {
        activeGrabberState = GrabberState.CUSTOM;
        targetGrabberPosition = position;
    }

    /**
     *
     * Systems Check
     */
    public void systemsCheck(AIMPad aimpad, Telemetry telemetry) {
        loop(aimpad);
        if (aimpad.isAPressed()) {
            setGrabberState(GrabberState.GRAB);
        } else if (aimpad.isBPressed()) {
            setGrabberState(GrabberState.RELEASE);
        } else if (aimpad.isYHeld()) {
            setGrabberStateCustom(aimpad.getRightStickY());
        }
    }
}
