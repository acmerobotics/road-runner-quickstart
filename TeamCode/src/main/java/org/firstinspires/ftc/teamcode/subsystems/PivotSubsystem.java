package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem extends SubsystemBase {

    private final DcMotorEx pivot;
    public double currentVelocity;
    public int currentPosition;
    private final Telemetry telemetry;

    public double angle;

    private final double Kg = 0.02;


    public PivotSubsystem(HardwareMap hwMap, @NonNull Telemetry telemetry){
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);

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
        return pivot.getCurrentPosition() * (1.0 / 2496 * 2 * Math.PI);
    }

//    public void setPosition(double angle){
//        pivot.setTargetPosition(math.cos((angle / (320/3)) * 2 * math.PI));
//    }

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

