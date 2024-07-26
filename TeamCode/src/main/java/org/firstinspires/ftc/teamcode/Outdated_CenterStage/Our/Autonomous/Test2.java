package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name="Test2", group="Linear OpMode")

@Disabled
public class Test2 extends LinearOpMode {

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

    //Sensors
    private IMU imu_IMU;
    //private ColorSensor colorSensor;

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
    double TrackWidth=4460;
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
/*
    Color Sensor
    boolean recognition = false;
    int gain=2;
    NormalizedRGBA normalizedColors;
    int Color;
    float hue;
    float saturation;
    float value;

 */

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



    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.75f;

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
        wrist2.setPosition(Ground);


        par0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        par1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        prev_left_encoder_pos= par0.getCurrentPosition();
        prev_right_encoder_pos=par1.getCurrentPosition();
        prev_center_encoder_pos= perp.getCurrentPosition();
    }

    private void Movement () {
        if (gamepad1.x) {
            // imu_IMU.resetYaw();

            par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


    private void Telemetry () {
        telemetry.addData("Change in X", x_pos);
        telemetry.addData("Change in Y", y_pos);
        telemetry.addData("Rotation", heading);

        telemetry.update();
    }

    @Override
    public void runOpMode() {

        InitialSetup();

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
            Yaw = .65f;

            deadwheels = (par1.getCurrentPosition()-par0.getCurrentPosition())/TrackWidth;

            heading += deadwheels;

            Speed_Movement_Multiplier=.75f;
            temp = Axial * Math.cos(TrackWidth) - Lateral * Math.sin(TrackWidth);
            Lateral = (float) (-Axial * Math.sin(TrackWidth) - Lateral * Math.cos(TrackWidth));
            Axial = (float) temp;

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


            if(gamepad1.a) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }

            //Sets wheel power
            leftFront.setPower((1 - Lateral + Yaw) * Speed_Movement_Multiplier);
            rightFront.setPower((1 + Lateral - Yaw) * Speed_Movement_Multiplier);
            leftBack.setPower((1 + Lateral + Yaw) * Speed_Movement_Multiplier);
            rightBack.setPower((1 -Lateral - Yaw) * Speed_Movement_Multiplier);

            Telemetry();
            Movement();
        }
    }
}
