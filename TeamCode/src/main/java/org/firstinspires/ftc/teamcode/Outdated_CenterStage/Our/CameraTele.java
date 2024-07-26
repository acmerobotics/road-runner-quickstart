package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name="CameraTele", group="Linear OpMode")

@Disabled

public class CameraTele extends LinearOpMode {

    //MOTORS
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor shoulder;
    private DcMotor par1;
    private DcMotor par0;

    //SERVOS
    private Servo claw;
    private Servo wrist;
    private Servo wrist2;
    private Servo claw2;
    private Servo air;

    //Sensors
    private IMU imu_IMU;
    //private ColorSensor colorSensor;

    //Speed
    float Speed_Movement_Multiplier;
    float RotationMultiplier;

    //Movement
    float Axial;
    float Lateral;
    float Yaw;
    double Gyro_Radians;
    double temp;
    double deadwheels;
    double TrackWidth=70;

    // Shoulder Variables
    double shoulderStick=0;
    int shoulder_Position=0;
    int PixelLevel = 1;
    int ShoulderLevel = 0;
    int clawInUse = 0;
    int shoulderFixer;

    //WRIST
    double Wrist;
    double InitWrist=0;
    double Ground=.216;

    /*Color Sensor
    boolean recognition = false;
    int gain=2;
    NormalizedRGBA normalizedColors;
    int Color;
    float hue;
    float saturation;
    float value;
     */

    //camera
    private static final String TFOD_MODEL_FILE = "CenterStage.tflite";
    int camera =0;
    private static final boolean USE_WEBCAM = true;
    private static final String[] LABELS = {
            "Pixel"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.4f;
        RotationMultiplier = .6f;

        //Wheel Setup
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        par1 = hardwareMap.get(DcMotor.class, "par1");
        par0 = hardwareMap.get(DcMotor.class, "par0");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class,"wrist");
        wrist2 = hardwareMap.get(Servo.class,"wrist2");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        //imu_IMU = hardwareMap.get(IMU.class, "imu");
        air = hardwareMap.get(Servo.class, "air");
        //colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");

        claw2.scaleRange(0,1);
        claw2.setPosition(.4);

        claw.setPosition(.4);
        claw.scaleRange(0, 1);

        air.setDirection(Servo.Direction.FORWARD);
        air.scaleRange(0, 1);
        air.setPosition(0);


        wrist2.setDirection(Servo.Direction.FORWARD);
        wrist2.scaleRange(0,1);
        wrist2.setPosition(0);


        par0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        par1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void Movement () {
        if (gamepad1.a) {
            Speed_Movement_Multiplier = 0.35f;
            RotationMultiplier = .6f;
        }
        if (gamepad1.b) {
            Speed_Movement_Multiplier = .7f;
            RotationMultiplier = 1f;
        }
        if (gamepad1.x) {
            // imu_IMU.resetYaw();

            par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y))>=1.7) {
            Speed_Movement_Multiplier=1f;
        }
        else if((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y))>=.4&&(Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y))<=1 ) {
            Speed_Movement_Multiplier=.6f;
        }
    }
    /*private void ColorSensor() {

        if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<7){
            recognition=true;
        }

        ((NormalizedColorSensor) colorSensor).setGain(gain);
        //telemetry.addData("Gain", ((NormalizedColorSensor) daLight).getGain());

        normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        telemetry.addData("Red", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3)));
        telemetry.addData("Green", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.green, 3)));
        telemetry.addData("Blue", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.blue, 3)));

        // Convert RGB values to Hue, Saturation, and Value.
        Color = normalizedColors.toColor();
        hue = JavaUtil.colorToHue(Color);
        saturation = JavaUtil.colorToSaturation(Color);
        value = JavaUtil.colorToValue(Color);

        if (hue <= 350 && hue >= 200) {
            recognition=true;
            //purple
        }
        else if(hue <= 190 && hue >= 110) {
            recognition=true;
            //green
        }
        else if(hue <= 100 && hue >= 45) {
            //yellow
            recognition=true;
        }
        if (saturation < 0.2) {
            telemetry.addData("Pixel Color", "White");
            //white
        }

        if(gamepad2.x && recognition){
            claw.setPosition(0);
            claw2.setPosition(1);
            if(claw.getPosition()==0 && claw2.getPosition()==1){
                recognition=false;
            }
        }


    }

     */
    private void Claws() {


        if( gamepad2.left_bumper && clawInUse >=0) {
            clawInUse -= 1;
            sleep(100);
        }
        else if(gamepad2.right_bumper && clawInUse <=0) {
            clawInUse +=1;
            sleep(100);
        }

        if (gamepad2.right_trigger > .2 && clawInUse ==0) {
            claw.setPosition(1);
            claw2.setPosition(0);
        }
        else if (gamepad2.left_trigger > .2 && clawInUse == 0) {
            claw.setPosition(.4);
            claw2.setPosition(.4);
        }
        //left claw only
        else if (gamepad2.right_trigger > .5 && clawInUse ==1) {
            claw.setPosition(1);
        }
        else if (gamepad2.left_trigger > .5 && clawInUse == 1) {
            claw.setPosition(.4);
        }
        //right claw only
        else if (gamepad2.right_trigger > .5 && clawInUse ==-1) {
            claw2.setPosition(0);
        }
        else if (gamepad2.left_trigger > .5 && clawInUse == -1) {
            claw2.setPosition(.4);
        }

    }
    private void Levels(){

        //Dpad Up
        if (gamepad2.dpad_up && PixelLevel ==1) {
            PixelLevel = 2;
            sleep(200);
        }
        else if (gamepad2.dpad_up && PixelLevel ==2) {
            PixelLevel = 3;
            sleep(200);
        }
        else if (gamepad2.dpad_up && PixelLevel ==3) {
            PixelLevel = 4;
            sleep(200);
        }
        else if (gamepad2.dpad_up && PixelLevel ==4) {
            PixelLevel = 5;
            sleep(200);
        }
        else if (gamepad2.dpad_up && PixelLevel ==5) {
            PixelLevel = 6;
            sleep(200);
        }


        //dpad down
        if (gamepad2.dpad_down && PixelLevel ==2) {
            PixelLevel = 1;
            sleep(200);
        }
        else if (gamepad2.dpad_down && PixelLevel ==3) {
            PixelLevel = 2;
            sleep(200);
        }
        else if (gamepad2.dpad_down && PixelLevel ==4) {
            PixelLevel = 3;
            sleep(200);
        }
        else if (gamepad2.dpad_down && PixelLevel ==5) {
            PixelLevel = 4;
            sleep(200);
        }

        else if (gamepad2.dpad_down && PixelLevel ==6) {
            PixelLevel = 5;
            sleep(200);
        }

    }
    private void Shoulder () {

        // shoulder using stick
        if (shoulderStick > 0) {
            shoulder_Position  += shoulderStick;
            shoulder.setTargetPosition(shoulder_Position);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);

        } else if (shoulderStick < 0 ) {
            shoulder_Position  += shoulderStick;
            shoulder.setTargetPosition(shoulder_Position);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(.3);
        } else {
            shoulder.setTargetPosition(shoulder_Position);
        }

        if (PixelLevel == 1) {
            ShoulderLevel=285;
            Wrist=Ground;
        }
        else if (PixelLevel == 2) {
            ShoulderLevel=340;
            Wrist=Ground+.02;
        }
        else if (PixelLevel == 3) {
            ShoulderLevel=415;
            Wrist=Ground+.05;
        }
        else if (PixelLevel == 4) {
            ShoulderLevel=505;
            Wrist=Ground+.1;
        }
        else if (PixelLevel == 5) {
            ShoulderLevel=590;
            Wrist=Ground+.15;
        }
        else if (PixelLevel ==6) {
            ShoulderLevel=620;
            Wrist=Ground+.17;
        }

        if (gamepad2.y) {
            wrist2.setPosition(Wrist);
            shoulder.setTargetPosition(ShoulderLevel);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(.8);
        }
        else if (gamepad2.a) {
            wrist2.setPosition(InitWrist);
            shoulder.setTargetPosition(shoulderFixer);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(.3);
            if(shoulder_Position <= 50)
            {
                shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        if(gamepad2.x) {
            wrist2.setPosition(Ground);
            shoulder.setTargetPosition(shoulderFixer);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(.3);
            if(shoulder_Position <= 50)
            {
                shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            List<Recognition> currentRecognitions = tfod.getRecognitions();

            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                double z= recognition.getImageWidth();
                if(x>=150 && x<300 && z>130) {
                    claw.setPosition(1);
                    claw2.setPosition(0);
                    if(claw.getPosition()>.9 || claw2.getPosition()<=.1) {
                        visionPortal.stopStreaming();
                    }
                }


            }
            currentRecognitions.size();
            visionPortal.stopStreaming();

        }
    }
    private void AirplaneModule () {
        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            air.setPosition(1);
        }
    }

    private void Telemetry () {
        telemetry.addData("Speed Movement Multiplier", Speed_Movement_Multiplier);
        telemetry.addData("Horizontal", Lateral);
        telemetry.addData("Axial", Axial);
        telemetry.addData("Temp", temp);
        telemetry.addData("Gameapd", gamepad1.right_stick_x);
        telemetry.addData("POWER", leftFront.getPower());
        telemetry.addData("Right Dead Wheel:", par1.getCurrentPosition());
        telemetry.addData("Left Dead Wheel:", par0.getCurrentPosition());
        telemetry.addData("Left Dead Wheel:", deadwheels);
        telemetry.addData("Servo", wrist2.getPosition());
        telemetry.addData("Servo", wrist.getPosition());
        telemetry.update();
    }
    private void Camera() {


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)


                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();


        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        builder.build();


    }

    @Override
    public void runOpMode() {

        InitialSetup();
        Camera();
        waitForStart();

        // Set Directions
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        par0.setDirection(DcMotorSimple.Direction.FORWARD);


        while (opModeIsActive()) {

            shoulderStick = -gamepad2.left_stick_y*60;
            shoulder_Position = shoulder.getCurrentPosition();

            Axial = -gamepad1.left_stick_y;
            Lateral = gamepad1.left_stick_x;
            Yaw = gamepad1.right_stick_x*.75f;

            deadwheels = (par1.getCurrentPosition()-par0.getCurrentPosition())/TrackWidth;
            Gyro_Radians = Math.toRadians(deadwheels);
            temp = Axial * Math.cos(Gyro_Radians) - Lateral * Math.sin(Gyro_Radians);
            Lateral = (float) (-Axial * Math.sin(Gyro_Radians) - Lateral * Math.cos(Gyro_Radians));
            Axial = (float) temp;

            //Sets wheel power
            leftFront.setPower((Axial - Lateral + Yaw) * Speed_Movement_Multiplier);
            rightFront.setPower((Axial + Lateral - Yaw) * Speed_Movement_Multiplier);
            leftBack.setPower((Axial + Lateral + Yaw) * Speed_Movement_Multiplier);
            rightBack.setPower((Axial -Lateral - Yaw) * Speed_Movement_Multiplier);

            Telemetry();
            Movement();
            Claws();
            Levels();
            Shoulder();
            AirplaneModule();
        }
    }
}
