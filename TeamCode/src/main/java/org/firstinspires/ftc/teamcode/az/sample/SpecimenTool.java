package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SpecimenTool extends LinearOpMode {
    private LinearOpMode opMode;
    Arm arm;
    Gripper gripper;
    Slides slides;

    public SpecimenTool(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode);
    }

    private void init(LinearOpMode opMode){
        arm = new Arm(opMode);
        gripper = new Gripper(opMode);
        slides = new Slides(opMode);
    }

    public void sampleDrop() {
        gripper.sampleDrop();
        sleep(500);
    }


    public void securePos() {
        //grab pixel
        gripper.collect();
        sleep(700);
        gripper.sampleDrop();
        sleep(100);

    }

    public void gripper_drop() {
        gripper.sampleDrop();
    }

    public void gripper_reset() {
        gripper.reset();
    }

    public void collect() {
        slides.collect();
        sleep(1000);
        gripper.collect();
        sleep(500);
        arm.collect();
        sleep(1000);
    }

    public void specimenHang() {
        slides.specimenHang();
        sleep(1000);
        gripper.specimenHang();
        sleep(500);
        arm.specimenHang();
        sleep(1000);
    }


    public void move() {
        gripper.move();
        arm.move();
        sleep(1000);
        slides.move();
        sleep(500);

    }

    public void specimenToolInit() {
        slides.move();
        sleep(500);
        arm.move();
        sleep(1000);
        gripper.move();


    }

    public void reset() {
        slides.reset();
        sleep(1000);
        arm.reset();
        sleep(500);
        gripper.reset();
        sleep(500);
    }

    public void halfwayReset() {
        slides.halfwayReset();
        sleep(500);
        gripper.reset();
        sleep(500);
        slides.reset();
        sleep(1000);
        arm.reset();
        sleep(500);
    }

    public void highReset () {
        slides.move();
        gripper.move();
        sleep(500);
        arm.move();
        sleep(1000);

    }

    public void arm_drop() {
       arm.drop();
    }

    //extend by a factor between 0 and 1
    public void extend(float factor) {slides.extend(factor);}

    public void slides_reset() {slides_reset();}

    @Override
    public void runOpMode() throws InterruptedException {

    }

}
