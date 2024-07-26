package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop.NeedToTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "LinearSlide")

@Disabled
//test comment JOHN aaa
public class LinearSlide extends LinearOpMode {
    private ElapsedTime wait = new ElapsedTime();
    private Servo ramp;
    private Servo bucket;
    private ColorSensor colorsens;
    private DcMotor slide1;
    private DcMotor slide2;
    private DcMotor intake;
    double pixelsseen;
    NormalizedRGBA normalizedColors;
    int color;
    float hue;
    float saturation;
    float value;
    int gain = 2;
    double slideinput;
    int slideposition;
    //values
    double bucketboardvalue = 0.083;
    double bucketintakevalue = 0;
    double rampopenvalue = 0;
    double rampclosedvalue = 0.25;
    double intakepower = 1;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            slideposition = slide1.getCurrentPosition();
            slideinput = -gamepad2.left_stick_y;
            intakesystem();
            slides();
            bucketcontrols();
        }
    }
    private void initialize() {
        colorsens = hardwareMap.get(ColorSensor.class, "colorsens");
        ramp = hardwareMap.get(Servo.class, "ramp");
        slide1 = hardwareMap.get(DcMotor.class, "slide1");
        slide2 = hardwareMap.get(DcMotor.class, "slide2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        bucket = hardwareMap.get(Servo.class, "bucket");
        bucket.scaleRange(0,1);
        ramp.scaleRange(0,1);
        bucket.setPosition(bucketintakevalue);
        ramp.setPosition(rampopenvalue);
        ramp.setDirection(Servo.Direction.REVERSE);
        bucket.setDirection(Servo.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        slide1.setDirection(DcMotorSimple.Direction.FORWARD);
        slide2.setDirection(DcMotorSimple.Direction.FORWARD);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideinput = 0;
    }
    private void intakesystemcolor(){
        if (gamepad2.a){
            pixelsseen = 0;
            intake.setPower(intakepower);
            while (opModeIsActive() && pixelsseen < 2 && !gamepad2.b){

                ((NormalizedColorSensor) colorsens).setGain(gain);
                normalizedColors = ((NormalizedColorSensor) colorsens).getNormalizedColors();
                color = normalizedColors.toColor();
                hue = JavaUtil.colorToHue(color);
                saturation = JavaUtil.colorToSaturation(color);

                if (hue <= 350 && hue >= 200) {
                    telemetry.addData("Pixel Color", "Purple");
                    pixelsseen = pixelsseen + 1;
                }
                else if(hue <= 190 && hue >= 110) {
                    telemetry.addData("Pixel Color", "Green");
                    pixelsseen = pixelsseen + 1;
                }
                else if(hue <= 100 && hue >= 45) {
                    telemetry.addData("Pixel Color", "Yellow");
                    pixelsseen = pixelsseen + 1;
                }
                else if(hue <= 30) {
                    telemetry.addData("Pixel Color", "White");
                    pixelsseen = pixelsseen + 1;
                }
                if (saturation < 0.2) {
                    telemetry.addData("Pixel Color", "White");
                    pixelsseen = pixelsseen + 1;
                }
                telemetry.addData("pixelsinbucket", pixelsseen);
                telemetry.update();
            }

            ramp.setPosition(rampclosedvalue);
            intake.setPower(0);
            telemetry.addData("bucket", "full");
            telemetry.update();
        }
    }
    private void intakesystem(){
        if (gamepad2.a){
            intake.setPower(intakepower);
            while (opModeIsActive() && !gamepad2.b){
                telemetry.addData("bucket", "filling");
            }
            ramp.setPosition(rampclosedvalue);
            intake.setPower(0);
            telemetry.addData("bucket", "full");
            telemetry.update();
        }
    }
    private void slides() {
        slideposition  += slideinput;
        slide1.setTargetPosition(slideposition);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide1.setPower(.5);
    }
    private void bucketcontrols(){
        pixeldrop();
        preparebucket();
        bucketreset();
    }
    private void pixeldrop() {
        if (gamepad2.right_trigger > .5 && gamepad2.left_trigger > .5) {
            ramp.setPosition(rampopenvalue);
        }
    }
    private void preparebucket(){
        if (gamepad2.y){
            bucket.setPosition(bucketboardvalue);
            ramp.setPosition(rampclosedvalue);
        }
    }
    private void bucketreset(){
        if (gamepad2.x){
            ramp.setPosition(rampopenvalue);
            bucket.setPosition(bucketintakevalue);
        }
    }
    private void waitfor() {
        wait.reset();
        while (wait.seconds() < time){
            telemetry.addData("Status", "waiting");
        }
    }
}