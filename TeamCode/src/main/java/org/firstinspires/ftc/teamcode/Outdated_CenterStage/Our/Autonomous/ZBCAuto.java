package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="BCAuto", group="Linear OpMode")

@Disabled
public class ZBCAuto extends LinearOpMode {

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
    double Multiply;
    int whichSide =0;

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

        ((DcMotorEx) BACK_L).setTargetPositionTolerance(20);
        ((DcMotorEx) BACK_R).setTargetPositionTolerance(20);
        ((DcMotorEx) FRONT_L).setTargetPositionTolerance(20);
        ((DcMotorEx) FRONT_R).setTargetPositionTolerance(20);
        ((DcMotorEx) shoulder).setTargetPositionTolerance(20);

        claw.setPosition(1);
    }
    private void IMU () {

        //Imu Initialize and Reset
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu_IMU.resetYaw();
    }

    private void Reset() {
        FRONT_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(FRONT_R.getCurrentPosition() <5 && FRONT_L.getCurrentPosition() <5 && BACK_L.getCurrentPosition() <5 && BACK_R.getCurrentPosition() <5) {
            Hasrun+=1;
        }
    }

    private void MoveForward (double DistanceInIn, double WheelSpeed) {

        FRONT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        if(FRONT_R.isBusy() && FRONT_L.isBusy() && BACK_L.isBusy() && BACK_R.isBusy())
        {

        }
        else{

            for (int i=0; i<1; i++) {
                Reset();
            }
        }

    }
    //find best way to drop pixel and hold the other one with one claw.




    private void MoveLeft (double DistanceInIn, double WheelSpeed) {

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

        if(FRONT_R.isBusy() && FRONT_L.isBusy() && BACK_L.isBusy() && BACK_R.isBusy())
        {
        }
        else{
            for (int i=0; i<1; i++) {
                Reset();
            }
        }
    }
    private void MoveRight (double DistanceInIn, double WheelSpeed) {

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

        if(FRONT_R.isBusy() && FRONT_L.isBusy() && BACK_L.isBusy() && BACK_R.isBusy())
        {

        }
        else{
            for (int i=0; i<1; i++) {
                Reset();
            }
        }
    }
    private void MoveBackward (double DistanceInIn, double WheelSpeed) {

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

        if(FRONT_R.isBusy() && FRONT_L.isBusy() && BACK_L.isBusy() && BACK_R.isBusy())
        {

        }
        else{
            for (int i=0; i<1; i++) {
                Reset();
            }
        }
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


        if(Gyro_Degrees<70 && Gyro_Degrees>20 && DistanceSensor<100&& DistanceSensor>20)
        {
            whichSide=1;

        } else if (Gyro_Degrees>-20 && Gyro_Degrees <20 && DistanceSensor<100 && DistanceSensor>20) {

            if(whichSide ==1) {

            }
            else {
                whichSide+=2;
            }
        } else {
            if(whichSide ==2 || whichSide ==1) {

            }
            else {
                whichSide+=3;
            }
        }

        Weirdthing = Gyro_Degrees + Angle;
        Multiply = Weirdthing/90;
        if (Weirdthing < 30 && Weirdthing>=15){

            FRONT_R.setPower(-.3);
            FRONT_L.setPower(.3);
            BACK_R.setPower(-.3);
            BACK_L.setPower(.3);

        }
        else if (Weirdthing < 15 && Weirdthing>=2){

            FRONT_R.setPower(-.25);
            FRONT_L.setPower(.25);
            BACK_R.setPower(-.25);
            BACK_L.setPower(.25);

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

            FRONT_R.setPower(-.25);
            FRONT_L.setPower(.25);
            BACK_R.setPower(-.25);
            BACK_L.setPower(.25);

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
    private void CloseClaw(int HowMuch) {
        claw.setPosition(HowMuch);
        if(claw.getPosition()==HowMuch)
        {
            Hasrun +=1;
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

            FRONT_R.setPower(.25);
            FRONT_L.setPower(-.25);
            BACK_R.setPower(.25);
            BACK_L.setPower(-.25);

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
        if(shoulder.isBusy())
        {

        }
        else{
            Hasrun +=1;
        }

    }
    private void LowerArm () {

        shoulder.setTargetPosition(-10);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);
        if(shoulder.isBusy())
        {
        }
        else{
            Hasrun +=1;
        }
    }



    private void Telemetry () {

        telemetry.addData("Tolerance", ((DcMotorEx) BACK_L).getTargetPositionTolerance());
        telemetry.addData("Tolerance", ((DcMotorEx) BACK_R).getTargetPositionTolerance());
        telemetry.addData("Tolerance", ((DcMotorEx) FRONT_R).getTargetPositionTolerance());
        telemetry.addData("Tolerance", ((DcMotorEx) FRONT_L).getTargetPositionTolerance());telemetry.addData("Gyro Degrees", Gyro_Degrees);
        telemetry.addData("WhichSide", whichSide);
        telemetry.addData("Tolerance", FRONT_R.getTargetPosition());
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
        claw.setPosition(1);
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


            if(Hasrun==0) {
                MoveForward(9,.8);
            }
            // 2. Rotate to Left
            if(Hasrun==1) {
                TurnLeft(60);
            }

            if(Hasrun == 2) {
                TurnRightLooking(60,.25);
            }

            if(Hasrun==3) {
                TurnLeft(0);
            }

            if(whichSide == 1 && Hasrun>=4) {
                if(Hasrun ==4) {
                    Reset();
                }

                if(Hasrun ==5) {
                    MoveForward(25,.7);
                }
                if(Hasrun ==6) {
                    Reset();
                }
                if(Hasrun ==7){
                    MoveLeft(12,.7);
                }
                if(Hasrun ==8) {
                    Reset();
                }

                //Release purple pixel
                if(Hasrun ==9) {
                    shoulder.setTargetPosition(-100);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setPower(1);
                    if(shoulder.isBusy())
                    {
                    }
                    else{
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==10) {
                    claw.setPosition(0);
                    if(claw.getPosition()==0)
                    {
                        Hasrun +=1;
                    }

                }
                if(Hasrun ==11) {
                    claw.setPosition(1);
                    if(claw.getPosition()==1)
                    {
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==12) {
                    shoulder.setTargetPosition(-200);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setPower(1);
                    if(shoulder.isBusy())
                    {
                    }
                    else{
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==13) {
                    MoveBackward(6,.6);
                }
                if(Hasrun ==14) {
                    Reset();
                }

                if(Hasrun==15) {
                    TurnLeft(90);
                }
                if(Hasrun ==16) {
                    Reset();
                }

                if(Hasrun==17) {
                    MoveForward(28,.9);
                }
                if(Hasrun ==18) {
                    Reset();
                }

                if(Hasrun==19) {
                    RaiseArm();
                }

                if(Hasrun ==20) {
                    MoveForward(5,.4);
                }
                if(Hasrun ==21) {
                    Reset();
                }
                if(Hasrun ==22) {
                    claw.setPosition(0);
                    if(claw.getPosition()==0)
                    {
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==23) {
                    MoveBackward(8,.3);
                }
                if(Hasrun ==24) {
                    Reset();
                }
                if(Hasrun == 25) {
                    LowerArm();
                }
                if(Hasrun == 26) {
                    TurnRight(0);
                }

                telemetry.addData("SpikeLocation","Left Spike");
            }
            if(whichSide ==2 && Hasrun>=4) {
                //if middle spike
                if(Hasrun ==4) {
                    Reset();
                }

                if(Hasrun ==5) {
                    MoveLeft(10,.9);
                }
                if(Hasrun ==6) {
                    Reset();
                }
                if(Hasrun ==7){
                    MoveForward(14,.9);
                }
                if(Hasrun ==8) {
                    Reset();
                }

                //Release purple pixel
                if(Hasrun ==9) {
                    shoulder.setTargetPosition(-100);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setPower(1);
                    if(shoulder.isBusy())
                    {
                    }
                    else{
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==10) {
                    claw.setPosition(0);
                    if(claw.getPosition()==0)
                    {
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==11) {
                    claw.setPosition(1);
                    if(claw.getPosition()==1)
                    {
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==12) {
                    shoulder.setTargetPosition(-200);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder.setPower(1);
                    if(shoulder.isBusy())
                    {
                    }
                    else{
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==13) {
                    MoveBackward(6,.6);
                }
                if(Hasrun ==14) {
                    Reset();
                }

                if(Hasrun==15) {
                    TurnLeft(90);
                }

                if(Hasrun==16) {
                    MoveForward(30,.9);
                }
                if(Hasrun ==17) {
                    Reset();
                }

                if(Hasrun==18) {
                    RaiseArm();
                }

                if(Hasrun ==19) {
                    MoveForward(5,.4);
                }
                if(Hasrun ==20) {
                    Reset();
                }
                if(Hasrun ==21) {
                    claw.setPosition(0);
                    if(claw.getPosition()==0)
                    {
                        Hasrun +=1;
                    }
                }
                if(Hasrun ==22) {
                    MoveBackward(8,.3);
                }
                if(Hasrun ==23) {
                    Reset();
                }
                if(Hasrun == 24) {
                    LowerArm();
                }
                if(Hasrun == 25) {
                    TurnRight(0);
                }

                telemetry.addData("SpikeLocation","Middle Spike");
            }

            if(whichSide ==3 && Hasrun>=4){


                switch (Hasrun) {
                    case 4: Reset();
                    break;
                    case 5: MoveForward(17,.6);
                    break;
                    case 6: TurnRight(90);
                        break;
                    case 7: MoveForward(10,.6);
                        break;
                    case 8: CloseClaw(0);
                        break;
                    case 9: MoveBackward(30,.6);
                        break;
                    case 10: TurnLeft(90);
                        break;
                    case 11: MoveForward(20,.8);
                        break;
                    case 12: MoveLeft(5,.9);
                        break;
                    case 13: RaiseArm();
                        break;
                    case 14: MoveForward(5,.4);
                        break;
                    case 15: CloseClaw(0);
                        break;
                    case 16: MoveBackward(10,.3);
                        break;
                    case 17: LowerArm();
                        break;
                    case 18: TurnRight(1);
                        break;
                    case 19: MoveBackward(10,.6);
                        break;
                    case 20: telemetry.addData("SpikeLocation","Right Spike");telemetry.addData("SpikeLocation","Right Spike");

                }


            }
        }



    }
}



