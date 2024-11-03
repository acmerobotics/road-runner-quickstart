package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class InitArmPosition extends LinearOpMode{

    Arm arm = null;
    LinearOpMode opMode;


    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;

        telemetry.addLine("Init");
        telemetry.update();

        Arm arm = new Arm(this);

        arm.setup();
        waitForStart();

        arm.moveToPosition(1300);
        sleep(6000);

    }
}
