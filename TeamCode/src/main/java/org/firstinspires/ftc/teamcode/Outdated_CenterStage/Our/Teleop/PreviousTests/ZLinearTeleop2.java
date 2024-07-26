package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop.PreviousTests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="ZLinearTeleop2", group="Linear OpMode")

@Disabled

public class ZLinearTeleop2 extends LinearOpMode {

    //Motor and Servo Assignment
    private DcMotor BACK_R;
    private DcMotor FRONT_L;
    private DcMotor FRONT_R;
    private DcMotor BACK_L;
    //claw is on the left, claw2 is on the right
    private DcMotor INTAKE;
    private DcMotor LinearSlides;
    private Servo air;
    private Servo bucket;

    //Sensors
    private IMU imu_IMU;

    //Speed Variable
    float Speed_Movement_Multiplier;

    //Gamepad 1 Input
    float Axial;
    float Lateral;
    float Yaw;

    //Wheel variables influenced by IMU
    AngularVelocity Angular_Velocity;
    YawPitchRollAngles Orientation2;
    double Gyro_Degrees;
    double Gyro_Radians;
    double temp;

    // Shoulder Variables
    double SlidesStick;
    int LinearSlidesPos;
    int PixelLevel = 1;
    int ShoulderLevel = 0;
    float RotationMultiplier;
    int shoulderFixer;
    DistanceSensor distance;


    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.4f;
        RotationMultiplier = .6f;

        //Wheel Setup
        BACK_L = hardwareMap.get(DcMotor.class, "BACK_L");
        BACK_R = hardwareMap.get(DcMotor.class, "BACK_R");
        FRONT_L = hardwareMap.get(DcMotor.class, "FRONT_L");
        FRONT_R = hardwareMap.get(DcMotor.class, "FRONT_R");
        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        LinearSlides = hardwareMap. get(DcMotor.class, "LinearSlides");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        bucket = hardwareMap. get(Servo.class, "bucket");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        air = hardwareMap.get(Servo.class, "air");

        bucket.scaleRange(0, 1);
        bucket.setPosition(0);
        air.scaleRange(0, 1);
        air.setPosition(0);

        shoulderFixer = -1*(LinearSlides.getCurrentPosition());


    }
    private void IMU () {

        //Imu Initialize and Reset
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu_IMU.resetYaw();
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
        //Reset Imu
        if (gamepad1.x) {
            imu_IMU.resetYaw();
        }


        if(gamepad1.right_trigger>.3 && PixelLevel ==1) {
            if(distance.getDistance(DistanceUnit.INCH)>30) {

                FRONT_L.setPower((Axial + 1 + Yaw) * .7);
                FRONT_R.setPower((Axial + 1 - Yaw) * .7);
                BACK_L.setPower((Axial + 1 + Yaw) * .7);
                BACK_R.setPower((Axial + 1 - Yaw) * .7);
            }
            else{
                FRONT_L.setPower(0);
                FRONT_R.setPower(0);
                BACK_R.setPower(0);
                BACK_L.setPower(0);
            }
        }
        if(gamepad1.right_trigger>.3  && PixelLevel ==2) {
            if(distance.getDistance(DistanceUnit.INCH)>40) {
                FRONT_L.setPower((Axial + 1 + Yaw) * .7);
                FRONT_R.setPower((Axial + 1 - Yaw) * .7);
                BACK_L.setPower((Axial + 1 + Yaw) * .7);
                BACK_R.setPower((Axial + 1 - Yaw) * .7);
            }
            else{
                FRONT_L.setPower(0);
                FRONT_R.setPower(0);
                BACK_R.setPower(0);
                BACK_L.setPower(0);
            }
        }
        if(gamepad1.right_trigger>.3  && PixelLevel ==3) {
            if(distance.getDistance(DistanceUnit.INCH)>50) {
                FRONT_L.setPower((Axial + 1 + Yaw) * .7);
                FRONT_R.setPower((Axial + 1 - Yaw) * .7);
                BACK_L.setPower((Axial + 1 + Yaw) * .7);
                BACK_R.setPower((Axial + 1 - Yaw) * .7);
            }
        }
        //Encoder must be added, don't test till it is added
        while(gamepad1.dpad_down) {
            FRONT_L.setPower((-1 - Lateral + Yaw) * .3);
            FRONT_R.setPower((1 + Lateral - Yaw) * .3);
            BACK_L.setPower((1 + Lateral + Yaw) * .3);
            BACK_R.setPower((-1 - Lateral - Yaw) * .3);
        }
        while(gamepad1.dpad_up) {
            FRONT_L.setPower((1 - Lateral + Yaw) * .3);
            FRONT_R.setPower((-1 + Lateral - Yaw) * .3);
            BACK_L.setPower((-1 + Lateral + Yaw) * .3);
            BACK_R.setPower((1 - Lateral - Yaw) * .3);
        }

    }
    private void LinearSlides() {
        if (SlidesStick > 0) {
            LinearSlidesPos  += SlidesStick;
            LinearSlides.setTargetPosition(LinearSlidesPos);
            LinearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearSlides.setPower(1);

        } else if (SlidesStick < 0 ) {
            LinearSlidesPos  += SlidesStick;
            LinearSlides.setTargetPosition(LinearSlidesPos);
            LinearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearSlides.setPower(1);
        } else {
            LinearSlides.setTargetPosition(LinearSlidesPos-=1);
        }

        //Pixel Level
        if (gamepad2.dpad_up && PixelLevel ==1) {
            PixelLevel = 2;
        }
        if (gamepad2.dpad_up && PixelLevel ==2) {
            PixelLevel = 3;
        }
        if (gamepad2.dpad_up && PixelLevel ==1) {
            PixelLevel = 4;
        }
        if (gamepad2.dpad_up && PixelLevel ==1) {
            PixelLevel = 5;
        }
        if (gamepad2.dpad_down && PixelLevel ==5) {
            PixelLevel = 4;
        }
        if (gamepad2.dpad_down && PixelLevel ==4) {
            PixelLevel = 3;
        }
        if (gamepad2.dpad_down && PixelLevel ==3) {
            PixelLevel = 2;
        }
        if (gamepad2.dpad_down && PixelLevel ==2) {
            PixelLevel = 1;
        }

        //Preset Level
        if (PixelLevel == 1) {
            ShoulderLevel=shoulderFixer+300;
        }
        if (PixelLevel == 2) {
            ShoulderLevel=shoulderFixer+500;
        }
        if (PixelLevel == 3) {
            ShoulderLevel=shoulderFixer+600;
        }
        if (PixelLevel == 4) {
            ShoulderLevel=shoulderFixer+700;
        }
        if (PixelLevel == 5) {
            ShoulderLevel=shoulderFixer+800;
        }

        //Preset Level Commands
        if (gamepad2.x) {
            LinearSlides.setTargetPosition(ShoulderLevel);
            LinearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearSlides.setPower(1);
        }
        else if (gamepad2.b) {
            LinearSlides.setTargetPosition(shoulderFixer);
            LinearSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearSlides.setPower(1);
        }

    }
    private void Bucket() {
        if (gamepad2.right_trigger > .2) {
            bucket.setPosition(1);
        }
        if (gamepad2.left_trigger > .2) {
            bucket.setPosition(0);
        }
    }
    private void Intake_Ramp() {

        if(gamepad2.a) {
            INTAKE.setPower(1);
            sleep(5000);
            INTAKE.setPower(0);
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
        telemetry.addData("Gyro Degrees", Gyro_Degrees);
        telemetry.addData("Gyro Radians", Gyro_Radians);
        telemetry.addData("Temp", temp);
        telemetry.addData("Yaw", Orientation2.getYaw(AngleUnit.DEGREES));
        telemetry.addData("FR", FRONT_R.getPower());
        telemetry.addData("FL", FRONT_L.getPower());
        telemetry.addData("BR", BACK_R.getPower());
        telemetry.addData("BL", BACK_L.getPower());
        telemetry.addData("shoulder", LinearSlides.getCurrentPosition());
        telemetry.addData("shoulderStick", SlidesStick);
        telemetry.addData("shoulder_Position", LinearSlidesPos);
        telemetry.addData("Pixel Level: ", PixelLevel);
        telemetry.addData("Shoulder Level:", ShoulderLevel);
        //telemetry.addData("shoulder", shoulder.getTargetPosition());
        telemetry.update();

    }

    @Override
    public void runOpMode() {

        InitialSetup();

        waitForStart();

        // Set Directions
        BACK_L.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_R.setDirection(DcMotorSimple.Direction.FORWARD);
        FRONT_L.setDirection(DcMotorSimple.Direction.REVERSE);
        FRONT_R.setDirection(DcMotorSimple.Direction.FORWARD);
        LinearSlides.setDirection(DcMotorSimple.Direction.FORWARD);
        LinearSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // shoulder Encoder
        LinearSlidesPos = 0;
        SlidesStick = 0;
        IMU();

        while (opModeIsActive()) {

            // Gamepad2 Variables
            SlidesStick = gamepad2.left_stick_y * 60;
            LinearSlidesPos = LinearSlides.getCurrentPosition();

            // Gamepad1 Variables
            Axial = -gamepad1.left_stick_y;
            Lateral = gamepad1.left_stick_x;
            Yaw = gamepad1.right_stick_x/Speed_Movement_Multiplier*.6f*RotationMultiplier;

            //Imu Variables
            Angular_Velocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            Orientation2 = imu_IMU.getRobotYawPitchRollAngles();

            // Mecanum Wheel Variables
            Gyro_Degrees = Orientation2.getYaw(AngleUnit.DEGREES);
            Gyro_Radians = Gyro_Degrees * Math.PI / 180;
            temp = Axial * Math.cos(Gyro_Radians) - Lateral * Math.sin(Gyro_Radians);
            Lateral = (float) (-Axial * Math.sin(Gyro_Radians) - Lateral * Math.cos(Gyro_Radians));
            Axial = (float) temp;

            //Sets wheel power
            FRONT_L.setPower((Axial - Lateral + Yaw) * Speed_Movement_Multiplier);
            FRONT_R.setPower((Axial + Lateral - Yaw) * Speed_Movement_Multiplier);
            BACK_L.setPower((Axial + Lateral + Yaw) * Speed_Movement_Multiplier);
            BACK_R.setPower((Axial - Lateral - Yaw) * Speed_Movement_Multiplier);

            // Telemetry
            Telemetry();

            // Gamepad1 COMPLETE
            Movement();
            Intake_Ramp();

            Bucket();

            LinearSlides();

            //Airplane COMPLETE
            AirplaneModule();

        }

    }


}
