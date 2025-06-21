package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pivot{
    DcMotor leftPivot, rightPivot;

    public Pivot(HardwareMap hardwareMap, Telemetry telemetry) {
        leftPivot = hardwareMap.get(DcMotor.class, "leftSlidePivot");
        rightPivot = hardwareMap.get(DcMotor.class, "rightSlidePivot");

        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class PivotToPosition implements Action {
        private int position;
        private boolean initialized = false;
        private final double power;
        private final int positionThreshold;

        public PivotToPosition(int position, double power, int positionThreshold) {
            this.position = position;
            this.power = power;
            this.positionThreshold = positionThreshold;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                leftPivot.setTargetPosition(position);
                rightPivot.setTargetPosition(position);

                leftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftPivot.setPower(power);
                rightPivot.setPower(power);

                initialized = true;
            }

            int leftError = Math.abs(leftPivot.getCurrentPosition() - position);
            int rightError = Math.abs(rightPivot.getCurrentPosition() - position);

            if (leftError > positionThreshold || rightError > positionThreshold) {
                return true; // Still moving
            }

            leftPivot.setPower(0);
            rightPivot.setPower(0);
            return false;
        }
    }

    public Action pivotToPosition(int position, double power, int positionThreshold) {
        return new PivotToPosition(position, power, positionThreshold);
    }
}