package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TestAutoSpecimenCollect extends BasicLeftAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initAuto();

        specimenTool.collect();

        sleep(5000);
    }
}
