package org.firstinspires.ftc.teamcode.New.SubSystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawJava {
    HardwareMap hardwareMap;
    Claw claw;
    ClawJava(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        claw = new Claw(hardwareMap);
    }

    public void update() {
        claw.update();
    }

}
