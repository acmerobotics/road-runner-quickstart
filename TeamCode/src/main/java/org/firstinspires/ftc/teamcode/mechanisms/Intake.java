package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    public CRServo intake;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    public static double INTAKE_COLLECT    = -1.0;
    public static double INTAKE_OFF        =  0.0;
    public static double INTAKE_DEPOSIT    =  0.5;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(INTAKE_OFF);
    }

    public void collect() {
        intake.setPower(INTAKE_COLLECT);
    }

    public void deposit() {
        intake.setPower(INTAKE_DEPOSIT);
    }

    public void stop() {
        intake.setPower(INTAKE_OFF);
    }
}
