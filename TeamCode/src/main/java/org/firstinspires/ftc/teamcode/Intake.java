package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Map;


public class Intake {
    private Telemetry telemetry;
    private ServoImplEx intake;


    public final String SYSTEM_NAME = "INTAKE";
    public final String SPIN_STATE = "SPINNING";
    public final String STOP_STATE = "STOPPED";

    public final double FULLY_OPEN_VALUE = 0.00;

    public ElapsedTime intakeCycleTime;


    private Map stateMap;

    public Intake (HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.intakeCycleTime = new ElapsedTime();

        intake = (ServoImplEx) hwMap.servo.get("Intake");
    }

    /************************* INTAKE UTILITIES **************************/

    // Opens the claw

    /*
    public void grabberOpen() {
        grabber.setPosition(CONE_OPEN_VALUE);
    }

    public void grabberClose() {
        grabber.setPosition(CLOSED_VALUE);
    }

    public static boolean shouldGrabberFullyOpen(Lift lift) {
        if (lift.getAvgPosition() > 300) {
            return true;
        }
        return false;
    }

    // Returns current position of the grabber. 0 is wide open (dropped cone)
    public double grabberPosition() {
        return grabber.getPosition();
    }

     */
}
