package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@TeleOp
@Autonomous
public class Slides extends LinearOpMode {

    DcMotorEx slideMotor1;
    DcMotorEx slideMotor2;
    LinearOpMode opMode;
    public static final double POWER = 1;
    public static final double EXTEND_POWER = 0.3;

    public static final int INCREMENT = 20;

    public Slides() {
        super();
    }

    public Slides(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }

    public void move() {
        setPos(SlidesPos.MOVE.value);
    }
    public void collect() {
        setPos(SlidesPos.COLLECT.value);

    }

    public void specimenHang() {
        setPos(SlidesPos.SPECIMEN_HANG.value);
    }

    public String printCurrentPos() {
       return  new StringBuffer().append("Slide 1: ")
                .append(slideMotor1.getCurrentPosition())
                .append(",\n Slide 2:")
                .append(slideMotor2.getCurrentPosition()).toString();
    }


    public enum SlidesPos {

        LEVEL_1_HANG(700),
        LEVEL_2_HANG(950),
        COLLECT(1000),

        MOVE(1000),
        SPECIMEN_HANG(2150),
        LEVEL_ONE_ASCENT(1800),
        LOWER_BASKET_DROP(1500),
        BASKET_DROP(3600),
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
        resetSlidePos();
    }

    private void resetSlidePos() {
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

    private void setPosLowPower(int pos){
        AZUtil.setBothMotorTargetPosition(slideMotor1, slideMotor2, pos, EXTEND_POWER);
    }

    private void setPosAndWait(int pos){
        setPos(pos);
        AZUtil.waitUntilMotorAtPos(this, slideMotor1, pos);
        AZUtil.waitUntilMotorAtPos(this, slideMotor2, pos);
    }

    public void extend(float factor) {
        int position = Math.round(SlidesPos.COLLECT.value + factor*1200);
        setPos(position);
    }

    public void reset() {
        setPos(SlidesPos.RESET.value);
        resetSlidePos();
    }

    public void resetAndWait() {
        setPosAndWait(SlidesPos.RESET.value);
    }

    public void halfwayReset() {
        setPos(SlidesPos.HALFWAYRESET.value);
    }


    public void moveToPosition(SlidesPos slidesPos){
        setPos(slidesPos.value);
    }
    public void moveToPositionLowPower(SlidesPos slidesPos){
        setPosLowPower(slidesPos.value);
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
            else if( gamepad1.dpad_down){
                moveDownSlider();
            }
        }
    }


    public void moveDownSlider() {

        if( getCurrentPos() > 800) {
            setPos(getCurrentPos() - 300);
        }
    }

    public void moveUpSlider() {
        if( getCurrentPos() < 3800) {
            setPos(getCurrentPos() + 300);
        }
    }


    private void autoMode() {

        telemetry.addLine("Init");
        telemetry.update();
        setup();

        waitForStart();

        while (opModeIsActive()) {
            setPos(SlidesPos.BASKET_DROP.value);
            sleep(5000);

            setPos(0);
            sleep(5000);
        }
    }
}
