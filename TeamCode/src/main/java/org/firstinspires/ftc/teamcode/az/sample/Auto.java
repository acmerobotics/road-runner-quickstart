package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Auto extends LinearOpMode {

    SpecimenTool specimenTool = new SpecimenTool(this);
    ElapsedTime runtime = new ElapsedTime();
    public void initAuto() {

        specimenTool.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();


    }
}
