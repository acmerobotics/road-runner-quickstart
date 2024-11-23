package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class DropTest extends LinearOpMode{

    Slides slides = null;
    Gripper gripper = null;
    LinearOpMode opMode;


    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;

        telemetry.addLine("Init");
        telemetry.update();

        Slides slides = new Slides(this);
        Gripper gripper = new Gripper(this);

        waitForStart();

        slides.moveToPosition(Slides.SlidesPos.BASKET_DROP);
        gripper.reset();
        sleep(10000);

        slides.reset();
        gripper.reset();
        sleep(4000);

    }
}
