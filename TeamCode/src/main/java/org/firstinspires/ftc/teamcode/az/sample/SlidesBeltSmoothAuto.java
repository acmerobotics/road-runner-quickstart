package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class SlidesBeltSmoothAuto extends SpecimenTool{
    public SlidesBeltSmoothAuto(LinearOpMode opMode) {
        super(opMode);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        init_loop();
        gripper.sampleDrop();
        waitForStart();

        while (opModeIsActive()){
            slides.moveToPosition(Slides.SlidesPos.BASKET_DROP);
            sleep(5000);
            slides.moveToPosition(Slides.SlidesPos.COLLECT);
            sleep(5000);
        }
    }
}
