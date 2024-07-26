package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="RCAUTO", group="Linear OpMode")

@Disabled
public class ZRCAuto extends LinearOpMode {

    //Motor and Servo Assignment
    private DcMotor BACK_R;
    private DcMotor FRONT_L;
    private DcMotor FRONT_R;
    private DcMotor BACK_L;
    private Servo claw;
    //private Servo claw2;
    private DcMotor shoulder;

    //Sensors
    private IMU imu_IMU;
    private DistanceSensor distance;

    //Speed Variable
    float Speed_Movement_Multiplier;

    //Wheel variables influenced by IMU
    AngularVelocity Angular_Velocity;
    YawPitchRollAngles Orientation2;
    double Gyro_Degrees;
    double Gyro_Radians;

    // Shoulder Variables
    double shoulderStick;
    int shoulder_Position;
    int PixelLevel = 1;
    int ShoulderLevel;

    //Encoder Movement
    double DistanceCorrection = 2.54;
    double DistanceInIn;
    double circumference = Math.PI * 9.6;
    double WheelSpeed;
    int encoderTarget;

    //IMU influenced Turn/Yaw
    int Angle;

    //Distance Sensor
    double actualDistance;
    double DistanceSensor;
    double Weirdthing;
    int Hasrun=0;
    int sleep = 0;
    double Multiply;

    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.4f;
        Angle = 0;

        //Wheel Setup
        BACK_L = hardwareMap.get(DcMotor.class, "BACK_L");
        BACK_R = hardwareMap.get(DcMotor.class, "BACK_R");
        FRONT_L = hardwareMap.get(DcMotor.class, "FRONT_L");
        FRONT_R = hardwareMap.get(DcMotor.class, "FRONT_R");
        shoulder = hardwareMap. get(DcMotor.class, "shoulder");
        claw = hardwareMap. get(Servo.class, "claw");
        //claw2 = hardwareMap.get(Servo.class, "claw2");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        claw.scaleRange(0, 1);
        //claw2.scaleRange(0,1);
        //claw2.setPosition(1);
        claw.setPosition(1);

        FRONT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRONT_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    private void IMU () {

        //Imu Initialize and Reset
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu_IMU.resetYaw();
    }

    private void MoveForward (double DistanceInIn, double WheelSpeed, int sleep) {
        actualDistance = DistanceInIn*DistanceCorrection;
        double rotations = actualDistance/circumference;
        int encoderTarget = (int) (rotations *537.6);


        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Hasrun +=1;
        sleep(sleep);

    }
    //find best way to drop pixel and hold the other one with one claw.




    private void MoveLeft (double DistanceInIn, double WheelSpeed, int sleep) {

        FRONT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        actualDistance = DistanceInIn*DistanceCorrection;
        double rotations = actualDistance/circumference;
        int encoderTarget = (int) (rotations *537.6);

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(-encoderTarget);
        BACK_R.setTargetPosition(-encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(sleep);
        Hasrun +=1;
    }
    private void MoveRight (double DistanceInIn, double WheelSpeed, int sleep) {

        FRONT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        actualDistance = DistanceInIn*DistanceCorrection;
        double rotations = actualDistance/circumference;
        int encoderTarget = (int) (rotations *537.6);

        FRONT_R.setTargetPosition(-encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(-encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(sleep);
        Hasrun +=1;
    }
    private void MoveBackward (double DistanceInIn, double WheelSpeed, int sleep) {

        FRONT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        actualDistance = DistanceInIn*DistanceCorrection;
        double rotations = actualDistance/circumference;
        int encoderTarget = (int) (rotations *537.6);

        FRONT_R.setTargetPosition(-encoderTarget);
        FRONT_L.setTargetPosition(-encoderTarget);
        BACK_R.setTargetPosition(-encoderTarget);
        BACK_L.setTargetPosition(-encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(sleep);
        Hasrun +=1;
    }
    private void TurnRightLooking (int Angle, double WheelSpeed) {
        FRONT_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FRONT_R.setPower(-WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(-WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        Weirdthing = Gyro_Degrees + Angle;
        Multiply = Weirdthing/90;
        if (Weirdthing < 30 && Weirdthing>=15){

            FRONT_R.setPower(-.3);
            FRONT_L.setPower(.3);
            BACK_R.setPower(-.3);
            BACK_L.setPower(.3);

        }
        else if (Weirdthing < 15 && Weirdthing>=2){

            FRONT_R.setPower(-.2);
            FRONT_L.setPower(.2);
            BACK_R.setPower(-.2);
            BACK_L.setPower(.2);

        }
        else if(Weirdthing < 5)
        {
            FRONT_R.setPower(0);
            FRONT_L.setPower(0);
            BACK_R.setPower(0);
            BACK_L.setPower(0);
            Hasrun +=1;
        }
    }

    private void TurnLeftLooking (int Angle, double WheelSpeed) {

        FRONT_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Weirdthing = Gyro_Degrees - Angle;

        Multiply = Weirdthing/90;
        if (Weirdthing > -30 && Weirdthing<= -15){

            FRONT_R.setPower(.3);
            FRONT_L.setPower(-.3);
            BACK_R.setPower(.3);
            BACK_L.setPower(-.3);

        }
        else if (Weirdthing > -15 && Weirdthing <= -2){

            FRONT_R.setPower(.2);
            FRONT_L.setPower(-.2);
            BACK_R.setPower(.2);
            BACK_L.setPower(-.2);

        }
        else if(Weirdthing > -5)
        {
            FRONT_R.setPower(0);
            FRONT_L.setPower(0);
            BACK_R.setPower(0);
            BACK_L.setPower(0);
            Hasrun +=1;
        }
    }



    private void TurnRight (int Angle) {
        FRONT_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FRONT_R.setPower(-WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(-WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        Weirdthing = Gyro_Degrees + Angle;
        Multiply = Weirdthing/90;
        if(Weirdthing >90)
        {
            FRONT_R.setPower(-.8);
            FRONT_L.setPower(.8);
            BACK_R.setPower(-.8);
            BACK_L.setPower(.8);
        }
        else if (Weirdthing <= 90 && Weirdthing>=30){

            FRONT_R.setPower(-Multiply);
            FRONT_L.setPower(Multiply);
            BACK_R.setPower(-Multiply);
            BACK_L.setPower(Multiply);

        }
        else if (Weirdthing < 30 && Weirdthing>=15){

            FRONT_R.setPower(-.3);
            FRONT_L.setPower(.3);
            BACK_R.setPower(-.3);
            BACK_L.setPower(.3);

        }
        else if (Weirdthing < 15 && Weirdthing>=2){

            FRONT_R.setPower(-.2);
            FRONT_L.setPower(.2);
            BACK_R.setPower(-.2);
            BACK_L.setPower(.2);

        }
        else if(Weirdthing < 5)
        {
            FRONT_R.setPower(0);
            FRONT_L.setPower(0);
            BACK_R.setPower(0);
            BACK_L.setPower(0);
        }
    }
    private void TurnLeft (int Angle) {
        FRONT_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Weirdthing = Gyro_Degrees - Angle;

        Multiply = Weirdthing/90;
        if(Weirdthing <= -100)
        {
            FRONT_R.setPower(.8);
            FRONT_L.setPower(-.8);
            BACK_R.setPower(.8);
            BACK_L.setPower(-.8);
        }
        else if (Weirdthing > -100 && Weirdthing<= -30){

            FRONT_R.setPower(-Multiply);
            FRONT_L.setPower(Multiply);
            BACK_R.setPower(-Multiply);
            BACK_L.setPower(Multiply);

        }
        else if (Weirdthing > -30 && Weirdthing<= -15){

            FRONT_R.setPower(.3);
            FRONT_L.setPower(-.3);
            BACK_R.setPower(.3);
            BACK_L.setPower(-.3);

        }
        else if (Weirdthing > -15 && Weirdthing <= -2){

            FRONT_R.setPower(.1);
            FRONT_L.setPower(-.1);
            BACK_R.setPower(.1);
            BACK_L.setPower(-.1);

        }
        else if(Weirdthing > -5)
        {
            FRONT_R.setPower(0);
            FRONT_L.setPower(0);
            BACK_R.setPower(0);
            BACK_L.setPower(0);
            Hasrun +=1;
        }
    }
    private void RaiseArm () {

        shoulder.setTargetPosition(-350);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);
        //where timer would start and eventually cause claw to release pixel
        sleep(1000);
        Hasrun +=1;



    }
    private void LowerArm () {

        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);
        sleep(1000);
        Hasrun +=1;

    }
    /*
    private void YellowPixelwDistanceSensor (double DistanceInIn, double WheelSpeed, int sleep) {

        //check if it constantly updates or is fixed
        double rotations = (DistanceSensor+20)/circumference;
        int encoderTarget = (int) (rotations *537.6);

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(sleep);
        Hasrun +=1;

        if (Hasrun == 11) {
            //move left or right depending on location
            Hasrun +=1;
        }
        if (Hasrun == 12) {
            PlaceYellowPixel();
        }



    }

     */

    private void InitateBigFunction () {
        // if(Gyro_Degrees<70 && Gyro_Degrees>20 && DistanceSensor<80) {}
        //

        if(Hasrun ==4) {
            MoveForward(12,.9,1000);
        }

        if(Hasrun ==5){
            MoveLeft(13,.9,1000);
        }

        //Release purple pixel
        if(Hasrun ==6) {
            shoulder.setTargetPosition(-100);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==7) {
            claw.setPosition(0);
            sleep(1000);
            Hasrun +=1;        }
        if(Hasrun ==8) {
            claw.setPosition(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==9) {
            shoulder.setTargetPosition(-200);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==10) {
            MoveBackward(6,.6,1000);
        }

        if(Hasrun==11) {
            TurnRight(90);
        }

        if(Hasrun==12) {
            MoveForward(28,.9,1000);
        }

        if(Hasrun==13) {
            RaiseArm();

        }
        if(Hasrun == 14) {
            MoveLeft(18,.7,1000);
        }
        if(Hasrun ==15) {
            MoveForward(5,.4,1000);
        }
        if(Hasrun ==16) {
            claw.setPosition(0);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==17) {
            MoveBackward(8,.3,1000);
        }
        if(Hasrun ==18) {
            LowerArm();
        }
        if(Hasrun == 19) {
            TurnLeft(0);
        }

        telemetry.addData("SpikeLocation","Left Spike");
/*
            //if middle spike
        } else if (Gyro_Degrees>-20 && Gyro_Degrees <20 && DistanceSensor < 80 ) {

         if(Hasrun ==4) {
            MoveRight(10,.9,1000);
        }
        if(Hasrun ==5){
            MoveForward(14,.9,1000);
        }

        //Release purple pixel
        if(Hasrun ==6) {
            shoulder.setTargetPosition(-100);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==7) {
            claw.setPosition(0);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==8) {
            claw.setPosition(1);\
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==9) {
            shoulder.setTargetPosition(-200);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==10) {
            MoveBackward(6,.6,1000);
        }

        if(Hasrun==11) {
            TurnRight(90);
        }

        if(Hasrun==12) {
            MoveForward(30,.9,1000);
        }

        if(Hasrun==13) {
            RaiseArm();
        }
        if(Hasrun ==14) {
            MoveRight(6,.7,1000);
        }

        if(Hasrun ==14) {
            MoveForward(5,.4,1000);
        }
        if(Hasrun ==15) {
            claw.setPosition(0);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==16) {
            MoveBackward(8,.3,1000);
        }
        if(Hasrun == 17) {
            LowerArm();
        }
        if(Hasrun == 18) {
            TurnLeft(0);
        }

        telemetry.addData("SpikeLocation","Middle Spike");

            //if right spike
        } else if (Gyro_Degrees<-30 && DistanceSensor < 70 ) {

        if(Hasrun ==4) {
            MoveForward(12,.9,1000);
        }
        if(Hasrun ==5){
            MoveRight(12,.9,1000);
        }

        //Release purple pixel
        if(Hasrun ==6) {
            shoulder.setTargetPosition(-100);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==7) {
            claw.setPosition(0);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==8) {
            claw.setPosition(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==9) {
            shoulder.setTargetPosition(-200);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==10) {
            MoveBackward(6,.6,1000);
        }

        if(Hasrun==11) {
            TurnRight(90);
        }

        if(Hasrun==12) {
            MoveForward(28,.9,1000);
        }

        if(Hasrun==13) {
            MoveRight(1,.9,1000);
        }

        if(Hasrun==14) {
            RaiseArm();
        }

        if(Hasrun ==15) {
            MoveForward(5,.4,1000);
        }
        if(Hasrun ==16) {
            claw.setPosition(0);
            sleep(1000);
            Hasrun +=1;
        }
        if(Hasrun ==17) {
            MoveBackward(8,.3,1000);
        }
        if(Hasrun == 18) {
            LowerArm();
        }
        if(Hasrun == 19) {
            TurnLeft(0);
        }
        telemetry.addData("SpikeLocation","Right Spike");

        }
        */

    }

    private void Telemetry () {

        telemetry.addData("Gyro Degrees", Gyro_Degrees);
        telemetry.addData("Gyro Radians", Gyro_Radians);
        telemetry.addData("Yaw", Orientation2.getYaw(AngleUnit.DEGREES));
        telemetry.addData("FR", FRONT_R.getPower());
        telemetry.addData("FL", FRONT_L.getPower());
        telemetry.addData("BR", BACK_R.getPower());
        telemetry.addData("BL", BACK_L.getPower());
        telemetry.addData("shoulder", shoulder.getCurrentPosition());
        telemetry.addData("shoulderStick", shoulderStick);
        telemetry.addData("shoulder_Position", shoulder_Position);
        telemetry.addData("Pixel Level: ", PixelLevel);
        telemetry.addData("Shoulder Level:", ShoulderLevel);
        telemetry.addData("Distance", DistanceInIn);
        telemetry.addData("Hasrun", Hasrun);
        telemetry.addData("Angle", Angle);
        telemetry.addData("WheelSpeed", WheelSpeed);
        telemetry.addData("Multuoly", Multiply);
        telemetry.addData("WeirdThing", Weirdthing);
        telemetry.addData("EncoderTarget", encoderTarget);
        telemetry.addData("Distance", FRONT_L.getMode());
        telemetry.addData("Distance", FRONT_R.getMode());
        telemetry.addData("Distance", BACK_L.getMode());
        telemetry.addData("Distance", BACK_R.getMode());
        //telemetry.addData("shoulder", shoulder.getTargetPosition());
        telemetry.update();

    }
    @Override
    public void runOpMode() {

        InitialSetup();

        // Set Directions
        BACK_L.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_R.setDirection(DcMotorSimple.Direction.FORWARD);
        FRONT_L.setDirection(DcMotorSimple.Direction.REVERSE);
        FRONT_R.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset shoulder Encoder
        shoulder_Position = 0;

        IMU();

        waitForStart();

        while (opModeIsActive()) {

            //Distance Sensor
            DistanceSensor = distance.getDistance(DistanceUnit.CM);

            //shoulderEncoder
            shoulder_Position = shoulder.getCurrentPosition();

            //Imu Angle Variables
            Angular_Velocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            Orientation2 = imu_IMU.getRobotYawPitchRollAngles();
            Gyro_Degrees = Orientation2.getYaw(AngleUnit.DEGREES);

            Telemetry();

            // 1. Move Forward
            if(Hasrun==0) {
                MoveForward(9,1,1000);
            }

            // 2. Rotate to Left
            if(Hasrun==1) {
                TurnRight(70);
            }

            if(Hasrun == 2) {
                TurnLeftLooking(70,.25);
            }

            if(Hasrun==3) {
                TurnRight(0);
            }
            if(Hasrun ==4) {
                InitateBigFunction();
            }



        }

    }


}
