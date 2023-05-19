package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem extends SubsystemBase {

    private final DcMotorEx pivot;
    public double currentVelocity;
    public int currentPosition;
    private final Telemetry telemetry;

    public double angle;
    private static final PIDFCoefficients pivotPID = new PIDFCoefficients(0.0,0.0,0.0,0.0);

    private final PIDFController pidf;

    private final double Kg = 0.02;


    public PivotSubsystem(HardwareMap hwMap, @NonNull Telemetry telemetry){
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        pivot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pivotPID);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

        pidf = new PIDFController(0,0,0,0);

        this.telemetry = telemetry;
    }

    public void setPower(double power){
        pivot.setPower(power + Kg * getAngle());
    }

    public double getVelocity(){
        currentVelocity = pivot.getVelocity();
        return currentVelocity;
    }

    public double getAngle() {
        return pivot.getCurrentPosition() * (1.0 / (5184 * 2 * Math.PI));
    }

    public double calculatePID(double desiredAngle) {
       return pidf.calculate(getAngle(), desiredAngle);
    }

    public void setAngle(double angle){
        pivot.setTargetPosition(angle);
    }

//    public int getPosition(){
//        currentPosition = pivot.getCurrentPosition();
//        return currentPosition;
//    }

    @Override
    public void periodic() {

        telemetry.addLine("Pivot")
                .addData("Encoder Ticks Pivot:", pivot.getCurrentPosition())
                .addData("Pivot Angle", getAngle());

        telemetry.update();
    }


}

