package org.firstinspires.ftc.teamcode.helpers.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoTest extends OpMode {

    Servo intakeLeft, intakeright, outtakeRotation, intakePivot, outtakePivot, intakeClaw, outtakeClaw;
    AnalogInput input;

    public static double intaketarget = 0.2;
    public static double outtaketarget = 1;
    public static double intakepivotTarget = 0;
    public static double outtakePivotTarget = 0.5;
    public static double outtakeClawTarget = 0.45;
    public static double intakeClawTarget = 0.35;

    // Scale factors based on observed values
    public static double MIN_ANALOG = 0.115; // Analog value when target position is 1
    public static double MAX_ANALOG = 0.905; // Analog value when target position is 0

    private boolean isOuttakePivotReversed = false;
    private boolean previousButtonAState = false;
    @Override
    public void init(){
        intakePivot = hardwareMap.get(Servo.class, "pivot");
        outtakePivot = hardwareMap.get(Servo.class, "outtakepivot");
        intakeLeft = hardwareMap.get(Servo.class, "armleft");
        intakeright = hardwareMap.get(Servo.class, "armright");
        input = hardwareMap.get(AnalogInput.class, "intake");
        intakeClaw = hardwareMap.get(Servo.class, "claw");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeclaw");
        outtakeRotation = hardwareMap.get(Servo.class, "outtakerotation");

        intakeLeft.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop(){
        double normalizedVoltage = mapVoltageToPosition(input.getVoltage()/3.3);

        intakeClaw.setPosition(intakeClawTarget);
        outtakeClaw.setPosition(outtakeClawTarget);
        outtakePivot.setPosition(outtakePivotTarget);
        intakePivot.setPosition(intakepivotTarget);

        intakeright.setPosition(intaketarget);
        intakeLeft.setPosition(intaketarget);

        outtakeRotation.setPosition(outtaketarget);

        telemetry.addData("Position", input.getVoltage()/3.3);
        telemetry.addData("Normalized Position:", normalizedVoltage);
    }

    // Method to map voltage to a normalized position between 0 and 1
    private double mapVoltageToPosition(double voltage) {
        // Map the voltage value to the range [0, 1]
        return Math.abs((voltage - MIN_ANALOG) / (MAX_ANALOG - MIN_ANALOG)  );
    }
}
