package org.firstinspires.ftc.teamcode;


import static java.lang.Math.atan;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="SwerveDriveJohn", group="Linear OpMode")

public class SwerveDriveJohn extends LinearOpMode {

    //MOTORS
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private Servo rightBackro;
    private Servo leftFrontro;
    private Servo rightFrontro;
    private Servo leftBackro;
    private DcMotor par1;
    private DcMotor par0;
    private DcMotor perp;

    RevBlinkinLedDriver blink;


    //Speed
    float Speed_Movement_Multiplier;

    //Movement
    float Axial;
    float Lateral;
    float Yaw;
    double temp;

    //Deadwheels
    double deadwheels;
    double TrackWidth=72;
    double heading;




    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.9f;

        //Wheel Setup
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFrontro = hardwareMap.get(Servo.class, "rightFrontro");
        leftFrontro = hardwareMap.get(Servo.class, "leftFrontro");
        rightBackro = hardwareMap.get(Servo.class, "rightBackro");
        leftBackro = hardwareMap.get(Servo.class, "leftBackro");
        par1 = hardwareMap.get(DcMotor.class, "par1");
        par0 = hardwareMap.get(DcMotor.class, "par0");
        perp = hardwareMap.get(DcMotor.class,"perp");

        rightFrontro.scaleRange(0,1);
        rightFrontro.setPosition(0);
        leftFrontro.scaleRange(0,1);
        leftFrontro.setPosition(0);
        rightBackro.scaleRange(0,1);
        rightBackro.setPosition(0);
        leftBackro.scaleRange(0,1);
        leftBackro.setPosition(0);


        blink = hardwareMap.get(RevBlinkinLedDriver.class, "blink");



        par0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        par1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        par0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        par1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


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
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        par0.setDirection(DcMotorSimple.Direction.FORWARD);


        while (opModeIsActive()) {



            deadwheels = (par1.getCurrentPosition()-par0.getCurrentPosition())/TrackWidth;
            double FinalGyro= Math.toRadians(deadwheels);
            heading = FinalGyro;

            Axial = -gamepad1.left_stick_y;
            Lateral = gamepad1.left_stick_x;
            Yaw = gamepad1.right_stick_x;

            Speed_Movement_Multiplier=.8f;
            temp = Axial * Math.cos(heading) - Lateral * Math.sin(heading);
            Lateral = (float) (-Axial * Math.sin(heading) - Lateral * Math.cos(heading));
            Axial = (float) temp;

            //Sets wheel power

            double angle = (Math.PI/2 + atan(Lateral/(Axial)));
            double angleadd = 0;
            if (Axial < 0){
                angleadd = Math.PI;
            }
            angle = angle+angleadd;
            //outputs the drive angle from 0-2pi

            double yawchangelf = Math.PI/2*(-Yaw+2.5);
            double yawchangerf = Math.PI/2*(Yaw+1.5);
            double yawchangelb = Math.PI/2*(-Yaw+1.5);
            double yawchangerb = Math.PI/2*(Yaw+2.5);
            //output is turning angle 0-2pi
            if (Yaw == 0){
                yawchangelf = angle;
                yawchangerf = angle;
                yawchangelb = angle;
                yawchangerb= angle;
            }

            double finalanglelf = (angle+yawchangelf)/2;
            double finalanglerf = (angle+yawchangerf)/2;
            double finalanglelb = (angle+yawchangelb)/2;
            double finalanglerb = (angle+yawchangerb)/2;
            //average the two angles together

            int directionlf = 1;
            if (finalanglelf > Math.PI){
                finalanglelf -= Math.PI;
                directionlf = -1;
            }
            int directionrf = 1;
            if (finalanglerf > Math.PI){
                finalanglerf -= Math.PI;
                directionrf = -1;
            }
            int directionlb = 1;
            if (finalanglelb > Math.PI){
                finalanglelb -= Math.PI;
                directionlb = -1;
            }
            int directionrb = 1;
            if (finalanglerb > Math.PI){
                finalanglerb -= Math.PI;
                directionrb = -1;
            }
            //reduces the angle the wheels can turn within 0-180 so its faster

            //range is 0-Math.PI whcih we change to 0-1 for the servo
            rightBackro.setPosition(finalanglerb/Math.PI);
            leftBackro.setPosition(finalanglelb/Math.PI);
            rightFrontro.setPosition(finalanglerf/Math.PI);
            leftFrontro.setPosition(finalanglelf/Math.PI);

            //apply directions
            leftFront.setPower(Speed_Movement_Multiplier*directionlf);
            rightFront.setPower(Speed_Movement_Multiplier*directionrf);
            leftBack.setPower(Speed_Movement_Multiplier*directionlb);
            rightBack.setPower(Speed_Movement_Multiplier*directionrb);

            Telemetry();
            Movement();

        }
    }
}
