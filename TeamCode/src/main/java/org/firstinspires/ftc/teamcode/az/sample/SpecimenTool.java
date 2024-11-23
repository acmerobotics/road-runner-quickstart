package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class SpecimenTool extends LinearOpMode {
    private LinearOpMode opMode;
    Arm arm;
    Gripper gripper;
    Slides slides;

    public SpecimenTool(){
        super();
    }
    public SpecimenTool(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode);
    }

    private void init(LinearOpMode opMode){
        arm = new Arm(opMode);
        gripper = new Gripper(opMode);
        slides = new Slides(opMode);
    }

    public void eject() {
        gripper.sampleDrop();
        //sleep(500);
    }

    public void dropHighBasket() {
        arm.setArmPos(Arm.ArmPos.BASKET_DROP);
        sleep(1200);
        slides.moveToPosition(Slides.SlidesPos.BASKET_DROP);
        gripper.dropPos();
    }

    public void levelOneAscent() {
        arm.setArmPos(Arm.ArmPos.LEVEL_ONE_ASCENT_PART_ONE);
        sleep(1200);
        slides.moveToPosition(Slides.SlidesPos.LEVEL_ONE_ASCENT);

        sleep(1000);
        arm.setArmPos(Arm.ArmPos.LEVEL_ONE_ASCENT);
    }


    public void printPos(Telemetry telemetry){
        telemetry.addData("Slide Pos", slides.printCurrentPos());
        telemetry.addData("Arm Pos:", arm.getCurrentPos());
        telemetry.update();
    }

    public void gripper_drop() {
        gripper.sampleDrop();
    }

    public void gripper_reset() {
        gripper.reset();
    }

    public void collect() {
        slides.collect();
//        sleep(500);
        gripper.collect();
//        sleep(500);
        arm.collect();
//        sleep(1000);
    }

    public void autoCollect()    {
        slides.collect();
//        sleep(500);
        gripper.autoCollect();
//        sleep(500);
        arm.autoCollect();
//        gripper.moveAround();

//        sleep(1000);
    }
 public void collectVertical() {
        slides.collect();
//        sleep(500);
        gripper.collectVertical();
//        sleep(500);
        arm.collect();
//        sleep(1000);
    }

    public void specimenHang() {
        arm.specimenHang();
        sleep(1000);
        slides.specimenHang();
        sleep(1000);
        gripper.specimenHang();
        sleep(500);

        arm.specimenDrop();
        sleep(500);
    }
 public void specimenLowBasket() {
        arm.specimenHang();
        sleep(1000);
        slides.specimenHang();
        sleep(1000);
        gripper.dropPos();
        sleep(500);
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
        sleep(1000);
        gripper.reset();
        sleep(500);
    }

    public void resetAndWait() {
        slides.resetAndWait();
        sleep(1000);
        arm.reset();
//        sleep(1000);
        gripper.reset();
//        sleep(500);
    }

    public void highReset () {
        slides.move();
        gripper.move();
        sleep(1000);
        arm.move();
        sleep(1000);

    }

    //extend by a factor between 0 and 1
    public void extend(float factor) {slides.extend(factor);}

    public void slides_reset() {slides_reset();}

    @Override
    public void runOpMode() throws InterruptedException {
        this.opMode = this;
        init(opMode);
        gripper.sampleDrop();
        waitForStart();

        while (opModeIsActive()){
            slides.moveToPosition(Slides.SlidesPos.BASKET_DROP);
            sleep(5000);
            slides.moveToPosition(Slides.SlidesPos.COLLECT);
            sleep(5000);
        }
    }

    public void level2Hang() {
        arm.setArmPos(Arm.ArmPos.LEVEL_TWO_HANG);
        sleep(500);
        slides.moveToPositionLowPower(Slides.SlidesPos.LEVEL_2_HANG);
        sleep(1500);
        slides.moveToPosition(Slides.SlidesPos.RESET);
        sleep(1000);
        arm.setArmPos(Arm.ArmPos.RESET);
        sleep(1000);
    }
}
