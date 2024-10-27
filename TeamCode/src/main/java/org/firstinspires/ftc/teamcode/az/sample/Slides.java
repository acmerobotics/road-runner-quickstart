package org.firstinspires.ftc.teamcode.az.sample;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Slides extends LinearOpMode {

    DcMotorEx slides;
    LinearOpMode opMode;

    public Slides() {
        super();
    }

    public Slides(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }

    public void move() {
        moveToPosition(SlidesPos.MOVE);
    }

    public enum SlidesPos {

        LEVEL_1_HANG(700),
        LEVEL_2_HANG(200),
        COLLECT(1000),

        MOVE(600),
        SPECIMEN_HANG(1500),
        LOWER_BASKET_DROP(1500),
        BASKET_DROP(3500),
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



    public void extend() {
        AZUtil.setMotorTargetPosition(slides, );
    }

    public void reset() {
        AZUtil.setMotorTargetPosition(slides, SlidesPos.RESET.value, .7);
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
