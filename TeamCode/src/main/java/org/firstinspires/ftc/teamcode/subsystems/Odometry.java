package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Odometry extends Mechanism {

    public DcMotorEx par0, par1, perp;

    @Override
    public void init(HardwareMap hwMap) {
        par0 = hwMap.get(DcMotorEx.class, "par0");
        par1 = hwMap.get(DcMotorEx.class, "par1");
        perp = hwMap.get(DcMotorEx.class, "perp");
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("par0 Position:", par0.getCurrentPosition());
        telemetry.addData("par1 Position:", par1.getCurrentPosition());
        telemetry.addData("perp Position:", perp.getCurrentPosition());
    }
}
