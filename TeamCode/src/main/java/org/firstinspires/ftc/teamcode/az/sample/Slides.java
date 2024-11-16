package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Slides extends LinearOpMode {

    DcMotorEx slideMotor1;
    DcMotorEx slideMotor2;
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
        AZUtil.setMotorTargetPosition(slideMotor1, SlidesPos.MOVE.value, POWER);
    }
    public void collect() {
        AZUtil.setMotorTargetPosition(slideMotor1, SlidesPos.COLLECT.value, POWER);
    }

    public void specimenHang() {
        AZUtil.setMotorTargetPosition(slideMotor1, SlidesPos.SPECIMEN_HANG.value, POWER);
    }
    public void specimenDrop() {
        AZUtil.setMotorTargetPosition(slideMotor1, SlidesPos.SPECIMEN_DROP.value, POWER);
    }

    public enum SlidesPos {

        LEVEL_1_HANG(700),
        LEVEL_2_HANG(200),
        COLLECT(1000),

        MOVE(600),
        SPECIMEN_HANG(2150),
        SPECIMEN_DROP(1800),
        LOWER_BASKET_DROP(1500),
        BASKET_DROP(3800),
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
        slideMotor1 = opMode.hardwareMap.get(DcMotorEx.class, "slides1");
        slideMotor2 = opMode.hardwareMap.get(DcMotorEx.class, "slides2");
        slideMotor1.setDirection(DcMotor.Direction.FORWARD);
        slideMotor2.setDirection(DcMotor.Direction.REVERSE);
        setMode();
    }

    private void setMode() {
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveUp(){
        int newPos = slideMotor1.getCurrentPosition() + INCREMENT;
        setPos(newPos);
    }

    public void moveDown(){
        int newPos = slideMotor1.getCurrentPosition() - INCREMENT;
        setPos(newPos);
    }
    private void setPos(int pos){
        AZUtil.setBothMotorTargetPosition(slideMotor1, slideMotor2, pos, POWER);
    }

    public void extend(float factor) {
        int position = Math.round(SlidesPos.COLLECT.value + factor*1200);
        setPos(position);
    }

    public void reset() {
        setPos(SlidesPos.RESET.value);
    }

    public void halfwayReset() {
        setPos(SlidesPos.HALFWAYRESET.value);
    }


    public void moveToPosition(SlidesPos slidesPos){
        setPos(slidesPos.value);
    }

    public int getCurrentPos(){
        return slideMotor1.getCurrentPosition();
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
        setPos(0);
    }

    public void moveUpSlider() {
        setPos(1000);
    }


    private void autoMode() {

        telemetry.addLine("Init");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            setup();
            sleep(2000);

            setPos(-40);
            sleep(10000);

            setPos(0);
            sleep(10000);
        }
    }
}
