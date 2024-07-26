package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.CompFiles;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name="AAAAAAAAAAAAA", group="Linear OpMode")

public class AAAAAAAAAAAAA extends LinearOpMode {

    //MOTORS
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor shoulder;
    private DcMotor par1;
    private DcMotor par0;
    private DcMotor perp;

    //SERVOS
    private Servo claw;
    private Servo wrist;
    private Servo wrist2;
    private Servo claw2;
    private Servo air;
    RevBlinkinLedDriver blink;

    //Sensors
    private IMU imu_IMU;
    private ColorSensor colorSensor;
    private ColorSensor colorSensor2;

    //Speed
    float Speed_Movement_Multiplier;

    //Movement
    float Axial;
    float Lateral;
    float Yaw;
    double Gyro_Radians;
    double temp;

    //Deadwheels
    double deadwheels;
    double TrackWidth=72;
    double prev_left_encoder_pos;
    double prev_right_encoder_pos;
    double prev_center_encoder_pos;
    double x_pos;
    double y_pos;
    double heading;

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
    double Ground=.218;

    // Color Sensor

    boolean ColorRecognition = false;
    boolean ColorRecognition2 = false;
    boolean DisRecognition = false;
    boolean DisRecognition2 = false;
    NormalizedRGBA normalizedColors;
    int Color;
    float hue;
    float saturation;
    float value;

    NormalizedRGBA normalizedColors2;
    int Color2;
    float value2;

    int gain = 2;

    //auto claw
    double finalClaw=1;
    double finalClaw2=.4;
    double initClaw=.67;
    double initClaw2=.83;


    private AprilTagProcessor aprilTag;
    private static final String TFOD_MODEL_FILE = "CenterStage.tflite";
    int camera =0;
    private static final boolean USE_WEBCAM = true;
    private static final String[] LABELS = {
            "Pixel"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    double distance;
    boolean Detection = false;
    double camX;
    double camY;
    double camYaw;
    double camR;


    int Mark1 = 40;
    double WithoutRot;

    //for 360 rotation
    int inMoved=0;
    double X;
    double YawSpeed;

    double finalAir=.4;




    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.9f;

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
        perp = hardwareMap.get(DcMotor.class,"perp");
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        colorSensor2 = hardwareMap.get(ColorSensor.class,"colorSensor2");

        blink = hardwareMap.get(RevBlinkinLedDriver.class, "blink");

        claw2.scaleRange(0,1);
        claw2.setPosition(initClaw2);

        claw.scaleRange(0, 1);
        claw.setPosition(initClaw);

        air.setDirection(Servo.Direction.FORWARD);
        air.scaleRange(0, 1);
        air.setPosition(0);


        wrist2.setDirection(Servo.Direction.FORWARD);
        wrist2.scaleRange(0,1);
        wrist2.setPosition(Ground);


        par0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        par1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    /*
   private void Camera() {

    aprilTag = new AprilTagProcessor.Builder()
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(true);

        builder.addProcessor(aprilTag);

        builder.build();
        /*

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
*/

    private void Movement () {
        if (gamepad1.x) {
            // imu_IMU.resetYaw();

            par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y))>.125&&(Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y))<=.4 ) {
            Speed_Movement_Multiplier=.4f;
        }
        else if((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y))>.4&&(Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y))<=.8 ) {
            Speed_Movement_Multiplier=.6f;
        }
        else  {
            Speed_Movement_Multiplier=.9f;
        }
    }
    private void Claws() {

        if(gamepad2.left_bumper && gamepad1.right_bumper) {
            clawInUse=0;
        }

        if( gamepad2.left_bumper && clawInUse >=0) {
            clawInUse -= 1;
            sleep(200);
        }
        else if(gamepad2.right_bumper && clawInUse <=0) {
            clawInUse +=1;
            sleep(200);
        }

        if (gamepad2.right_trigger > .2 && clawInUse ==0) {
            claw.setPosition(finalClaw);
            claw2.setPosition(finalClaw2);
        }
        else if (gamepad2.left_trigger > .2 && clawInUse == 0) {
            claw.setPosition(initClaw);
            claw2.setPosition(initClaw2);
        }
        //left claw only
        else if (gamepad2.right_trigger > .5 && clawInUse ==1) {
            claw.setPosition(finalClaw);
        }
        else if (gamepad2.left_trigger > .5 && clawInUse == 1) {
            claw.setPosition(initClaw);
        }
        //right claw only
        else if (gamepad2.right_trigger > .5 && clawInUse ==-1) {
            claw2.setPosition(finalClaw2);
        }
        else if (gamepad2.left_trigger > .5 && clawInUse == -1) {
            claw2.setPosition(initClaw2);
        }

    }
        private void ColorSensor() {

        blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        if(((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)<3){
            DisRecognition=true;
        }
        else{
            DisRecognition=false;
        }

        if(((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM)<3){
            DisRecognition2=true;
        }
        else{
            DisRecognition2=false;
        }

        ((NormalizedColorSensor) colorSensor).setGain(2);
        normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        //Convert RGB values to Hue, Saturation, and Value.
        Color = normalizedColors.toColor();
        hue = JavaUtil.colorToHue(Color);
        saturation = JavaUtil.colorToSaturation(Color);
        value = JavaUtil.colorToValue(Color);

        //Color Sensor 2 (Left claw)
        ((NormalizedColorSensor) colorSensor2).setGain(2);
        normalizedColors2 = ((NormalizedColorSensor) colorSensor2).getNormalizedColors();
        Color2 = normalizedColors2.toColor();
        value2 = JavaUtil.colorToValue(Color2);

       /* if (hue <= 350 && hue >= 200&& saturation!=0 || value != 0) {
            ColorRecognition=true;
            //purple
        }
        else if(hue <= 160 && hue >= 120 && (saturation!=0 || value != 0)) {
            ColorRecognition=true;
            //green
        }
        else if(hue <= 100 && hue >= 45&& saturation!=0 || value != 0) {
            //yellow
            ColorRecognition=false;
        }
        if (saturation > 0.35) {
            ColorRecognition=false;
            //white
        }

        */

            if(value>= .07) {
                ColorRecognition=true;
            }
            if(value2>= .07) {
                ColorRecognition2=true;
            }


        if(gamepad2.x && ColorRecognition&&DisRecognition){
            claw.setPosition(finalClaw);
            if(claw.getPosition()==finalClaw){
                ColorRecognition=false;
                DisRecognition=false;
            }
        }

        if(gamepad2.x && ColorRecognition2&&DisRecognition2){
                claw2.setPosition(finalClaw2);
                if(claw2.getPosition()==finalClaw2){
                    ColorRecognition2=false;
                    DisRecognition2=false;
                }
        }

        if((claw.getPosition()>.6 && claw.getPosition()<.68) || (claw2.getPosition()>.8 &&claw2.getPosition()<.88 )) {
            blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else {
            blink.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        /*
        if (claw2.getPosition()==finalClaw2&&claw.getPosition()==finalClaw && value>=.07 && value2>=.7) {
            wrist2.setPosition(0);
        }

         */


        }

    private void Levels(){
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
        /*
        if (gamepad2.dpad_down && PixelLevel ==1) {
            PixelLevel = 0;
            sleep(200);
        }
        */
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

        /*  shoulder using stick
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
*/      /*
        if (PixelLevel == 0) {
            ShoulderLevel=140;
            Wrist=Ground+0.11;
        }
        */

        double WristPos = 0;
        int ShoulderPos=0;


        switch (PixelLevel){

            case 1:
                ShoulderLevel=315;
                Wrist=Ground;
                distance=5;

                WristPos=.326;
                ShoulderPos = 212;
                break;
            case 2:
                ShoulderLevel=375;
                Wrist=Ground+.02;
                distance=4;
                WristPos=.31;
                ShoulderPos = 190;
                break;
            case 3:
                ShoulderLevel=435;
                Wrist=Ground+.05;

                distance=3.2;
                break;
            case 4:
                ShoulderLevel=525;
                Wrist=Ground+.1;

                distance=2.8;
                break;
            case 5:
                ShoulderLevel=590;
                Wrist=Ground+.15;

                distance=2;
                break;
            case 6:
                ShoulderLevel=620;
                Wrist=Ground+.17;

                distance=1.5;
            case 9:
                break;
        }

        if (gamepad2.y) {
            wrist2.setPosition(Wrist);
            shoulder.setTargetPosition(ShoulderLevel);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
        }
        else{
            shoulder.setTargetPosition(shoulder_Position);
        }
        /*if (gamepad2.b) {
            wrist2.setPosition(WristPos);
            shoulder.setTargetPosition(ShoulderPos);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
        }


        else{
            shoulder.setTargetPosition(shoulder_Position);
        }

         */
        if (gamepad2.a) {
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

            /*

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

             */

        }
    }
    private void AirplaneModule () {
        if (gamepad1.y) {
            air.setPosition(finalAir);
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
        telemetry.addData("Heading:", heading);
        telemetry.addData("Servo", wrist2.getPosition());
        telemetry.addData("Servo", wrist.getPosition());

        telemetry.addData("Servo", wrist.getPosition());
        telemetry.addData("Hue", Double.parseDouble(JavaUtil.formatNumber(hue, 0)));
        telemetry.addData("Saturation", Double.parseDouble(JavaUtil.formatNumber(saturation, 3)));
        telemetry.addData("Value", Double.parseDouble(JavaUtil.formatNumber(value, 3)));
        telemetry.addData("Value2", Double.parseDouble(JavaUtil.formatNumber(value2, 3)));

        telemetry.addData("Value2", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        telemetry.addData("Value", ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM));

        telemetry.update();
    }
    private void CameraOP()
    {
        visionPortal.resumeStreaming();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                for (int i = 0; i<1; i+=1) {
                    camX= detection.ftcPose.x;
                    camY= detection.ftcPose.y-8;
                    camYaw = detection.ftcPose.yaw;
                    camR = detection.ftcPose.range;
                    Detection = true;
                }
            }
        }
    }
    @Override
    public void runOpMode() {

        InitialSetup();

        //Camera();

        waitForStart();

        while (opModeIsActive()) {

            shoulderStick = -gamepad2.left_stick_y*60;
            shoulder_Position = shoulder.getCurrentPosition();

            deadwheels = (par1.getCurrentPosition()-par0.getCurrentPosition())/TrackWidth;
            double FinalGyro= Math.toRadians(deadwheels);
            heading = FinalGyro;

            Axial = -gamepad1.left_stick_y;
            Lateral = gamepad1.left_stick_x;
            Yaw = gamepad1.right_stick_x*.65f;

            Speed_Movement_Multiplier=.8f;
            temp = Axial * Math.cos(heading) - Lateral * Math.sin(heading);
            Lateral = (float) (-Axial * Math.sin(heading) - Lateral * Math.cos(heading));
            Axial = (float) temp;

            //Sets wheel power
            leftFront.setPower((Axial - Lateral + Yaw) * Speed_Movement_Multiplier);
            rightFront.setPower((Axial + Lateral - Yaw) * Speed_Movement_Multiplier);
            leftBack.setPower((Axial + Lateral + Yaw) * Speed_Movement_Multiplier);
            rightBack.setPower((Axial - Lateral - Yaw) * Speed_Movement_Multiplier);

            ColorSensor();
/*
            if(gamepad1.right_trigger>.5){
                CameraOP();
                if(Detection) {
                    for (int i=0; i<1; i+=1) {
                        visionPortal.stopStreaming();
                        prev_left_encoder_pos= par0.getCurrentPosition();
                        prev_right_encoder_pos=par1.getCurrentPosition();
                        prev_center_encoder_pos= perp.getCurrentPosition();
                    }

                    double left_encoder_pos= par0.getCurrentPosition();
                    double right_encoder_pos=par1.getCurrentPosition();
                    double center_encoder_pos= perp.getCurrentPosition();

                    double delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
                    double delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
                    double delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

                    double delta_middle_pos = ((delta_left_encoder_pos + delta_right_encoder_pos) / 2);
                    double delta_perp_pos = (delta_center_encoder_pos - (-1322.4924337170135 * deadwheels));

                    double delta_x = (delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading));
                    double delta_y = (delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading));

                    x_pos += delta_x;
                    y_pos += delta_y;

                    if(Math.abs(x_pos)>=camY) {

                        Speed_Movement_Multiplier=.75f;

                        //Lateral, Axial, and Rotational

                        WithoutRot = camX/camY;

                        X= (inMoved * camYaw)/360;
                        YawSpeed= X/(Math.sqrt(Math.pow(camY,2)+Math.pow(camX,2)));

                            temp = WithoutRot * Math.cos(TrackWidth) - Math.sin(TrackWidth);
                            Lateral = (float) (-WithoutRot * Math.sin(TrackWidth) - Math.cos(TrackWidth));
                            Axial = (float) temp;

                        leftFront.setPower((Axial - Lateral + YawSpeed) * Speed_Movement_Multiplier);
                        rightFront.setPower((Axial + Lateral - YawSpeed) * Speed_Movement_Multiplier);
                        leftBack.setPower((Axial + Lateral + YawSpeed) * Speed_Movement_Multiplier);
                        rightBack.setPower((Axial -Lateral - YawSpeed) * Speed_Movement_Multiplier);


                    }
                    else {
                        temp = Axial * Math.cos(TrackWidth) - Lateral * Math.sin(TrackWidth);
                        Lateral = (float) (-Axial * Math.sin(TrackWidth) - Lateral * Math.cos(TrackWidth));
                        Axial = (float) temp;

                        leftFront.setPower((Axial - Lateral + Yaw) * Speed_Movement_Multiplier);
                        rightFront.setPower((Axial + Lateral - Yaw) * Speed_Movement_Multiplier);
                        leftBack.setPower((Axial + Lateral + Yaw) * Speed_Movement_Multiplier);
                        rightBack.setPower((Axial -Lateral - Yaw) * Speed_Movement_Multiplier);

                        Speed_Movement_Multiplier=1f;
                    }
                }

            }
            else {
                Speed_Movement_Multiplier=.9f;
                temp = Axial * Math.cos(TrackWidth) - Lateral * Math.sin(TrackWidth);
                Lateral = (float) (-Axial * Math.sin(TrackWidth) - Lateral * Math.cos(TrackWidth));
                Axial = (float) temp;



                leftFront.setPower((Axial + Lateral + Yaw) * Speed_Movement_Multiplier);
                rightFront.setPower((Axial - Lateral - Yaw) * Speed_Movement_Multiplier);
                leftBack.setPower((Axial - Lateral + Yaw) * Speed_Movement_Multiplier);
                rightBack.setPower((Axial +Lateral - Yaw) * Speed_Movement_Multiplier);

            }

*/
            //Sets wheel power

            Telemetry();
            Movement();
            Claws();
            Levels();
            Shoulder();
            AirplaneModule();
        }
    }
}
