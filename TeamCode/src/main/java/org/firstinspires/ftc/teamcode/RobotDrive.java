package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotDrive {
    DcMotorEx lf; // Port 0 CH
    DcMotorEx lb; // Port 1 CH
    DcMotorEx rb; // Port 2 CH
    DcMotorEx rf; // Port 3 CH

    public DcMotorEx initDcMotor(HardwareMap hardwareMap, String name, DcMotor.Direction dir) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(dir);
        return motor;
    }

    public void init(HardwareMap hardwareMap) {
        lf = initDcMotor (hardwareMap, "lf", DcMotor.Direction.REVERSE);
        rf = initDcMotor (hardwareMap, "rf", DcMotor.Direction.FORWARD);
        lb = initDcMotor (hardwareMap, "lb", DcMotor.Direction.REVERSE);
        rb = initDcMotor (hardwareMap, "rb", DcMotor.Direction.FORWARD);

    }
    public void driveXYW(double rx, double ry, double rw) {
        double lfPower = rx - ry - rw;
        double rfPower= rx + ry + rw;
        double lbPower= rx + ry - rw;
        double rbPower = rx - ry + rw;
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }
}