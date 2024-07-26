package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="EncoderTest", group="Linear OpMode")

@Disabled
public class ZEncoderTest extends LinearOpMode {

    //Motor and Servo Assignment
    private DcMotor BACK_R;
    private DcMotor FRONT_L;
    private DcMotor FRONT_R;
    private DcMotor BACK_L;


    //Sensors
    private IMU imu_IMU;

    //Speed Variable
    float Speed_Movement_Multiplier;

    //Wheel variables influenced by IMU
    AngularVelocity Angular_Velocity;
    YawPitchRollAngles Orientation2;
    double Gyro_Degrees;
    double Gyro_Radians;




    //Encoder Movement
    double DistanceInCM;
    double circumference = Math.PI * 9.6;
    double WheelSpeed;
    int StepNum = 1;
    int encoderTarget;
    double rotations;
    int Hasrun =0;

    //IMU influenced Turn/Yaw

    private void InitialSetup () {


        Speed_Movement_Multiplier = 0.4f;

        //Wheel Setup
        BACK_L = hardwareMap.get(DcMotor.class, "BACK_L");
        BACK_R = hardwareMap.get(DcMotor.class, "BACK_R");
        FRONT_L = hardwareMap.get(DcMotor.class, "FRONT_L");
        FRONT_R = hardwareMap.get(DcMotor.class, "FRONT_R");
        imu_IMU = hardwareMap.get(IMU.class, "imu");

        FRONT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRONT_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }
    private void IMU () {

        //Imu Initialize and Reset
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu_IMU.resetYaw();
    }

    private void MoveForward (double DistanceInCM, double WheelSpeed) {

        double rotations = DistanceInCM/circumference;
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

        sleep(3000);

        Hasrun+=1;
    }

    private void MoveBackward (double DistanceInCM) {

        double rotations = DistanceInCM/circumference;
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

    }


    private void Telemetry () {
        telemetry.addData("Hasrun", Hasrun);
        telemetry.addData("Gyro Degrees", Gyro_Degrees);
        telemetry.addData("Gyro Radians", Gyro_Radians);
        telemetry.addData("Yaw", Orientation2.getYaw(AngleUnit.DEGREES));
        telemetry.addData("FR", FRONT_R.getPower());
        telemetry.addData("FL", FRONT_L.getPower());
        telemetry.addData("BR", BACK_R.getPower());
        telemetry.addData("BL", BACK_L.getPower());
        telemetry.addData("Encoder Target", encoderTarget);
        telemetry.addData("Distance",DistanceInCM);
        telemetry.addData("Rotations", rotations);
        telemetry.addData("Circumference", circumference);

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

        // Reset shoulder Encoder


        IMU();

        waitForStart();

        while (opModeIsActive()) {

            //shoulderEncoder


            //Imu Angle Variables
            Angular_Velocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            Orientation2 = imu_IMU.getRobotYawPitchRollAngles();
            Gyro_Degrees = Orientation2.getYaw(AngleUnit.DEGREES);

// Telemetry
            Telemetry();

            // 1. Move Forward
            StepNum = 1;

            WheelSpeed = .8;
            if(Hasrun==0) {
                MoveForward(30,.8);
            }
            if(Hasrun==1) {
                MoveBackward(5);
            }

            telemetry.addData("Step: ",StepNum);
            telemetry.addData("Should go: ", "twelve inches");




        }

    }


}
