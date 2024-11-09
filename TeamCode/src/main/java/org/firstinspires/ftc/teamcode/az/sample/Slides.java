package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Slides extends LinearOpMode {

    DcMotorEx slides;
    LinearOpMode opMode;
    public static final double POWER = 1;
    public static final int INCREMENT = 20;

    public Slides() {
        super();
    }

    public Slides(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }

    public void move() {
        AZUtil.setMotorTargetPosition(slides, SlidesPos.MOVE.value, POWER);
    }
    public void collect() {
        AZUtil.setMotorTargetPosition(slides, SlidesPos.COLLECT.value, POWER);
    }

    public void specimenHang() {
        AZUtil.setMotorTargetPosition(slides, SlidesPos.SPECIMEN_HANG.value, POWER);
    }

    public enum SlidesPos {

        LEVEL_1_HANG(700),
        LEVEL_2_HANG(200),
        COLLECT(1000),

        MOVE(600),
        SPECIMEN_HANG(1500),
        LOWER_BASKET_DROP(1500),
        BASKET_DROP(3700),
        HALFWAYRESET(700),
        RESET(0);


        private int value;

        SlidesPos(int val) {
            this.value = val;
        }

        public double getValue() {
            return this.value;
        }
    }

    public void setup() {
        slides = opMode.hardwareMap.get(DcMotorEx.class, "slides");
        slides.setDirection(DcMotor.Direction.FORWARD);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveUp(){
        int newPos = slides.getCurrentPosition() + INCREMENT;
        setPos(newPos);
    }

    public void moveDown(){
        int newPos = slides.getCurrentPosition() - INCREMENT;
        setPos(newPos);
    }
    public void setPos(int pos){
        AZUtil.setMotorTargetPosition(slides, pos, POWER);
    }



    public void extend(float factor) {
        int position = Math.round(SlidesPos.COLLECT.value + factor*1200);
        AZUtil.setMotorTargetPosition(slides, position, POWER);
    }

    public void reset() {
        AZUtil.setMotorTargetPosition(slides, SlidesPos.RESET.value, POWER);
    }

    public void halfwayReset() {
        AZUtil.setMotorTargetPosition(slides, SlidesPos.HALFWAYRESET.value, POWER);
    }


    public void moveToPosition(SlidesPos slidesPos){
        AZUtil.setMotorTargetPosition(slides, slidesPos.value, .7);
    }

    public int getCurrentPos(){
        return slides.getCurrentPosition();
    }
    @Override
    public void runOpMode() {
        this.opMode = this;

        telemetry.addLine("Init");
        telemetry.update();
        setup();

        waitForStart();

//        teleOp();
        autoMode();

    }

    private void teleOp() {
        while (opModeIsActive()){
            if (gamepad1.dpad_up){
                moveUpSlider();
            }
            else {
                moveDownSlider();
            }
        }
    }


    public void moveDownSlider() {
        AZUtil.setMotorTargetPosition(slides, 0, 1);
    }

    public void moveUpSlider() {
        AZUtil.setMotorTargetPosition(slides, 1000, 1);
    }

    private void autoMode() {
        AZUtil.setMotorTargetPosition(slides, 800, 1);
        sleep(10000);

        AZUtil.setMotorTargetPosition(slides, 0, 1);
        sleep(10000);
    }
}
