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

@Autonomous(name="AngleTest", group="Linear OpMode")

@Disabled
public class ZAngleTest extends LinearOpMode {

    //Motor and Servo Assignment
    private DcMotor BACK_R;
    private DcMotor FRONT_L;
    private DcMotor FRONT_R;
    private DcMotor BACK_L;

    //Sensors
    private IMU imu_IMU;

    //Wheel variables influenced by IMU
    AngularVelocity Angular_Velocity;
    YawPitchRollAngles Orientation2;
    double Gyro_Degrees;

    // Shoulder Variables
    int shoulder_Position;

    //Encoder Movement
    double WheelSpeed;
    int StepNum = 1;

    //IMU influenced Turn/Yaw
    int Angle;
    double Weirdthing;
    int Hasrun =1;
    double Multiply;

    //Distance Sensor

    private void InitialSetup () {

        Angle = 0;

        //Wheel Setup
        BACK_L = hardwareMap.get(DcMotor.class, "BACK_L");
        BACK_R = hardwareMap.get(DcMotor.class, "BACK_R");
        FRONT_L = hardwareMap.get(DcMotor.class, "FRONT_L");
        FRONT_R = hardwareMap.get(DcMotor.class, "FRONT_R");
        imu_IMU = hardwareMap.get(IMU.class, "imu");


        FRONT_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    private void IMU () {

        //Imu Initialize and Reset
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu_IMU.resetYaw();
    }

    private void TurnRight (int Angle) {

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
            Hasrun +=1;
        }
    }
    private void TurnLeft (int Angle) {

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(-WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(-WheelSpeed);

        Weirdthing = Gyro_Degrees - Angle;

        Multiply = Weirdthing/90;

        if(Weirdthing <= -90)
        {
            FRONT_R.setPower(.8);
            FRONT_L.setPower(-.8);
            BACK_R.setPower(.8);
            BACK_L.setPower(-.8);
        }
        else if (Weirdthing > -90 && Weirdthing<= -30){

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

    private void Telemetry () {

        telemetry.addData("Step: ",WheelSpeed);
        telemetry.addData("Yaw", Orientation2.getYaw(AngleUnit.DEGREES));
        telemetry.addData("FR", FRONT_R.getPower());
        telemetry.addData("FL", FRONT_L.getPower());
        telemetry.addData("BR", BACK_R.getPower());
        telemetry.addData("BL", BACK_L.getPower());
        telemetry.addData("Angle", Angle);
        telemetry.addData("WeirdThing", Weirdthing);
        telemetry.addData("has",Hasrun);
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
        shoulder_Position = 0;

        IMU();

        waitForStart();

        while (opModeIsActive()) {

            //Imu Angle Variables
            Angular_Velocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            Orientation2 = imu_IMU.getRobotYawPitchRollAngles();
            Gyro_Degrees = Orientation2.getYaw(AngleUnit.DEGREES);
            Telemetry();

            // 1. Turn Right

            StepNum = 1;
            if(Hasrun ==1) {
                TurnLeft(90);
            }
            if(Hasrun ==2) {
                TurnRight(90);
            }
            if(Hasrun ==3) {
                TurnLeft(90);
            }



            telemetry.addData("Step: ",StepNum);

            /* 2. Turn Left
            StepNum = 1;
            WheelSpeed = 1;
            Angle = -90;

            TurnLeft();
            telemetry.addData("Step: ",StepNum);
*/



        }

    }


}
