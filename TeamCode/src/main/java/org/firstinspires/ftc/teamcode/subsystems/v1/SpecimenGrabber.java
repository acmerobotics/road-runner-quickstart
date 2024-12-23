package org.firstinspires.ftc.teamcode.subsystems.v1;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.settings.ConfigurationInfo;

public class SpecimenGrabber extends Mechanism {

    Servo specimenGrabber;
    //states of specimen grabber
    public enum GrabberState {
        GRAB, RELEASE, CUSTOM
    }

    GrabberState activeGrabberState = GrabberState.RELEASE;

    double targetGrabberPosition;

    // setting grab position
    private final static double GRAB_POSITION = 0.0;

    // setting release position
    private final static double RELEASE_POSITION =  0.21;

    /**
     * initializing hardware
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        specimenGrabber = hwMap.get(Servo.class, ConfigurationInfo.specimenGrabber.getDeviceName());
    }

    /**
     * loop to be called each systems cycle
     * @param aimPad references AIMPad in slot one
     */
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

    /**
     * set target position to grab
     */
    public void grab() {
        targetGrabberPosition = GRAB_POSITION;
    }

    /**
     * set target position to release
     */
    public void release() {
        targetGrabberPosition = RELEASE_POSITION;
    }

    /**
     * set grabber state to input state
     * @param state
     */
    public void setGrabberState(GrabberState state) {
        activeGrabberState = state;
    }

    /**
     * sets grabber to target input position
     * @param position
     */
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
