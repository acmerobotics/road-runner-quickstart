package org.firstinspires.ftc.teamcode.New.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    HardwareMap hardwareMap;
    DcMotorEx claw;
    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        claw = hardwareMap.get(DcMotorEx.class, "claw");
    }

    public void update() {
        claw.setPower(1);
    }

}
