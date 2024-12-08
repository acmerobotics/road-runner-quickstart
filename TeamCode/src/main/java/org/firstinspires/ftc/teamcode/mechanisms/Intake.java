package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    public CRServo intake;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    public static double INTAKE_COLLECT    = 1.0;
    public static double INTAKE_OFF        =  0.0;
    public static double INTAKE_DEPOSIT    =  -0.5;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(INTAKE_OFF);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
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

    // auto
    public class DepositAuto implements Action {
        // initialize variables
        private boolean initialized = false;
        private double beginTs = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            // one-time things inside this if statement
            if (!initialized){
                beginTs = Actions.now();
                initialized = true;
            }
            // the following code runs over and over until the function returns false.
            duration = Actions.now() - beginTs;
            packet.put("duration", duration);
            packet.put("power ", intake.getPower());
            if (duration <1){
                intake.setPower(INTAKE_DEPOSIT);
                return true;
            } else {
                intake.setPower(INTAKE_OFF);
                return false;
            }
        }
    }

    public Action depositAction() {
        return new DepositAuto();
    }


    public class IntakeAuto implements Action {
        // initialize variables
        private boolean initialized = false;
        private double beginTs = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double duration;
            // one-time things inside this if statement
            if (!initialized){
                beginTs = Actions.now();
                initialized = true;
            }
            // the following code runs over and over until the function returns false.
            duration = Actions.now() - beginTs;
            packet.put("duration", duration);
            packet.put("power ", intake.getPower());
            if (duration < 2.0){
                intake.setPower(INTAKE_COLLECT);
                return true;
            } else {
                intake.setPower(INTAKE_OFF);
                return false;
            }
        }
    }

    public Action intakeAction() {
        return new IntakeAuto();
    }

}
