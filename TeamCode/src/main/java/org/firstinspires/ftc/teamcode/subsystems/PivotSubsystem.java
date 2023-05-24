package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem extends SubsystemBase {

    private final DcMotorEx pivot;
    private final Telemetry telemetry;

    private double currentAngle;

    public static PIDFCoefficients kPIDF = new PIDFCoefficients(0,0,0,0);
    public final PIDFController controller;
    private double desiredAngle;

    private double power;

    private final double Kg = 0.02;


    public PivotSubsystem(HardwareMap hwMap, @NonNull Telemetry telemetry){
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(1,0,0,0);
        this.telemetry = telemetry;
        this.desiredAngle = desiredAngle;
    }

    public void setPower(double power){
        pivot.setPower(power + Kg * getAngle());
    }

    public double getAngle() {
        currentAngle = (pivot.getCurrentPosition() * ((22 * 2 * Math.PI) / (28 * 81 * 66)));
        return currentAngle;
    }

    public void setAngle(double angle){
        desiredAngle = angle;
    }

    public void calculatePID() {
        power = controller.calculate(getAngle(), desiredAngle);
        pivot.setPower(power);
    }

    @Override
    public void periodic() {
        calculatePID();
        telemetry.addLine("Pivot")
                .addData("Encoder Ticks Pivot:", pivot.getCurrentPosition())
                .addData("Pivot Angle", getAngle());
        telemetry.update();
    }

}

