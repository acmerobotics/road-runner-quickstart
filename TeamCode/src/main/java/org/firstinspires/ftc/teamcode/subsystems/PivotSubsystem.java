package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.MathUtil;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PivotSubsystem extends SubsystemBase {

    private final DcMotorEx pivot;
    private final Telemetry telemetry;

    public static PIDFController kPIDF = new PIDFController(1,0,0,0.2);
    private double desiredAngle;

    public static double tolerance = 0.2;
    public static double kG = 0.2;


    public PivotSubsystem(@NonNull HardwareMap hwMap, @NonNull Telemetry telemetry){
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kPIDF.setTolerance(tolerance);

        this.telemetry = telemetry;
    }

    public void setPower(double power){
        pivot.setPower(power);
    }

    public double getAngle() {
        return pivot.getCurrentPosition() * ((22 * 2 * Math.PI) / (28 * 81 * 66));
    }

    public void setAngle(double angle){
        desiredAngle = angle;
    }

    public double calculatePID() {
        return kPIDF.calculate(getAngle(), desiredAngle) + Math.sin(getAngle()) * kG;
    }

    public boolean atSetpoint() {
        return kPIDF.atSetPoint();
    }

    @Override
    public void periodic() {
        setPower(calculatePID());
        telemetry.addLine("Pivot")
                .addData("\nEncoder Ticks Pivot:", pivot.getCurrentPosition())
                .addData("\nPivot Angle", getAngle());
        telemetry.update();
    }

}

