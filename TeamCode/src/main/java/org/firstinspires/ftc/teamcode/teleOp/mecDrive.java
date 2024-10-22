package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum Drive")
public class mecDrive extends OpMode {
    //Driving Variables
    public static final double driveSpeed = 0.66;
    public static final double fastSpeed = 1.0;
    public static final double slowSpeed = 0.25;
    public static double finalSlowmode = 0.0;
    public static boolean slowMode = true;
    public static int gyroAdj = 0;

    //Lifting and Servo Variables
    int liftPos = 0;
    int liftPosAdj = 0;
    double liftPower = 0.0;
    double clawPos = 0.0;
    double servoRot = 0.0;
    double outtakeRot = 0.0;
    boolean startHanging = false;

    MecanumDrive mecanumDrive;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    //Create servos
    Servo launchServo;
    Servo clawServo;
    Servo clawRotateServo;
    Servo outtakeServo;

    //Create the magnet sensors
    TouchSensor touch1;
    TouchSensor touch2;
    TouchSensor slideLimit;

    //Create distance sensors
    DistanceSensor left;
    DistanceSensor right;
    DistanceSensor back;

    //Create the other motors
    DcMotor hanger;
    DcMotor slideMotor;

    //Create the variable that will keep track of the left joystick's x value
    public float leftstick_x = 0;
    public float g1_leftstick_x = 0;
    public float g2_leftstick_x = 0;

    //Create the variable that will keep track of the left joystick's y value
    public float leftstick_y = 0;
    public float g1_leftstick_y = 0;
    public float g2_leftstick_y = 0;

    //Create the variable that will keep track of the right joystick's x value
    public float rightstick_x = 0;
    public float g1_rightstick_x = 0;
    public float g2_rightstick_x = 0;

    //Create the variable that will keep track of the right joystick's y value
    public float rightstick_y = 0;
    public float g1_rightstick_y = 0;
    public float g2_rightstick_y = 0;

    //Create the variable that tracks GamePad1 buttons
    public boolean g1_dpad_down = false;
    public boolean g1_dpad_up = false;
    public boolean g1_dpad_right = false;
    public boolean g1_dpad_left = false;

    //Create the variable that tracks GamePad2 buttons
    public boolean g2_dpad_down = false;
    public boolean g2_dpad_up = false;
    public boolean g2_dpad_right = false;
    public boolean g2_dpad_left = false;

    // GamePad bumpers
    public boolean g1_left_bumper = false;
    public boolean g1_right_bumper = false;
    public boolean g2_right_bumper = false;
    public boolean g2_left_bumper = false;

    //GamePad triggers
    public float g1_right_trigger = 0;
    public float g1_left_trigger = 0;
    public float g2_right_trigger = 0;
    public float g2_left_trigger = 0;

    //GamePad buttons
    public boolean g1_a = false;
    public boolean g1_b = false;
    public boolean g1_x = false;
    public boolean g1_y = false;
    public boolean g2_a = false;
    public boolean g2_b = false;
    public boolean g2_x = false;
    public boolean g2_y = false;

    //Create the gyroscope
    public IMU imu;

    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;

    public void init() {
        //Add the motors to the configuration on the phones
        leftFront = new Motor(hardwareMap, "LF");
        rightFront = new Motor(hardwareMap, "RF");
        leftBack = new Motor(hardwareMap, "LB");
        rightBack = new Motor(hardwareMap, "RB");

        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void loop() {
        getController();
        mecanumDrive();
    }

    public void getController() {
        //Gamepad joysticks
        g1_leftstick_x = gamepad1.left_stick_x;
        g2_leftstick_x = gamepad2.left_stick_x;

        g1_leftstick_y = gamepad1.left_stick_y;
        g2_leftstick_y = gamepad2.left_stick_y;

        g1_rightstick_x = gamepad1.right_stick_x;
        g2_rightstick_x = gamepad2.right_stick_x;

        g1_rightstick_y = gamepad1.right_stick_y;
        g2_rightstick_y = gamepad2.right_stick_y;


        //Directional pad buttons
        g1_dpad_down = gamepad1.dpad_down;
        g1_dpad_up = gamepad1.dpad_up;
        g1_dpad_right = gamepad1.dpad_right;
        g1_dpad_left = gamepad1.dpad_left;
        g2_dpad_down = gamepad2.dpad_down;
        g2_dpad_up = gamepad2.dpad_up;
        g2_dpad_right = gamepad2.dpad_right;
        g2_dpad_left = gamepad2.dpad_left;

        //Gamepad buttons
        g1_a = gamepad1.a;
        g1_b = gamepad1.b;
        g1_x = gamepad1.x;
        g1_y = gamepad1.y;
        g2_a = gamepad2.a;
        g2_b = gamepad2.b;
        g2_x = gamepad2.x;
        g2_y = gamepad2.y;

        //Gamepad bumpers
        g1_right_bumper = gamepad1.right_bumper;
        g1_left_bumper = gamepad1.left_bumper;
        g2_right_bumper = gamepad2.right_bumper;
        g2_left_bumper = gamepad2.left_bumper;

        //Gamepad triggers
        g1_right_trigger = gamepad1.right_trigger;
        g1_left_trigger = gamepad1.left_trigger;
        g2_right_trigger = gamepad2.right_trigger;
        g2_left_trigger = gamepad2.left_trigger;
    }

    public void mecanumDrive(){

        //Setting boolean hold
        if(g1_right_bumper) {
            //Slowmode
            finalSlowmode = slowSpeed;

        } else if (g1_left_bumper) {
            //Fastmode
            finalSlowmode = fastSpeed;
        } else {
            //Regular
            finalSlowmode = driveSpeed;
        }

        if (g1_y){
            imu.resetYaw();
        }

        orientation = imu.getRobotYawPitchRollAngles();
        mecanumDrive.driveFieldCentric(gamepad1.left_stick_x * finalSlowmode, -gamepad1.left_stick_y * finalSlowmode, gamepad1.right_stick_x * finalSlowmode, orientation.getYaw(AngleUnit.DEGREES));
    }
}
