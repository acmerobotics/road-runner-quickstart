package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="AACompetitionTELEOPDOUBLE2", group="Linear OpMode")

@Disabled

public class AACompetitionTELEOPDOUBLE2 extends LinearOpMode {

    //Motor and Servo Assignment
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    //claw is on the left, claw2 is on the right
    private Servo claw;
    private Servo claw2;
    private Servo wrist;
    private DcMotor shoulder;
    private Servo air;

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
    double shoulderStick;
    int shoulder_Position;
    int PixelLevel = 1;
    int ShoulderLevel = 0;
    int clawInUse = 0;
    float RotationMultiplier;
    int shoulderFixer;
    DistanceSensor distance;
    double BrSpeed=1;
    double BlSpeed=1;
    double FlSpeed=1;
    double FrSpeed=1;
    ElapsedTime time= new ElapsedTime();
    double wristlevel = 0;


    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.4f;
        RotationMultiplier = .6f;

        //Wheel Setup
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class,"claw2");
        wrist = hardwareMap.get(Servo.class,"wrist");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        air = hardwareMap.get(Servo.class, "air");
        claw2.setPosition(0);
        claw2.scaleRange(0,1);
        claw.setPosition(0);
        claw.scaleRange(0, 1);
        air.scaleRange(0, 1);
        air.setPosition(0);
        wrist.setPosition(0);
        wrist.scaleRange(0,1);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

    }

    private void Attachment () {

        // Claw
        if (gamepad2.right_trigger > .2 && clawInUse ==0) {
            claw.setPosition(1);
            claw2.setPosition(1);
        }
        if (gamepad2.left_trigger > .2 && clawInUse == 0) {
            claw.setPosition(0);
            claw2.setPosition(0);
        }


        //left claw only
        if (gamepad2.right_trigger > .5 && clawInUse ==-1) {
            claw.setPosition(1);
        }
        if (gamepad2.left_trigger > .5 && clawInUse == -1) {
            claw.setPosition(-1);
        }

        //right claw only
        if (gamepad2.right_trigger > .5 && clawInUse ==1) {
            claw2.setPosition(1);
        }
        if (gamepad2.left_trigger > .5 && clawInUse == 1) {
            claw2.setPosition(0);
        }


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
            shoulder.setPower(.5);
        } else {
            shoulder.setTargetPosition(shoulder_Position+1);
        }

        //Pixel Level
        if (gamepad2.dpad_up && PixelLevel ==1) {
            PixelLevel = 2;
            sleep(200);
        }
        if (gamepad2.dpad_up && PixelLevel ==2) {
            PixelLevel = 3;
            sleep(200);
        }
        if (gamepad2.dpad_down && PixelLevel ==3) {
            PixelLevel = 2;
            sleep(200);
        }
        if (gamepad2.dpad_down && PixelLevel ==2) {
            PixelLevel = 1;
            sleep(200);
        }


        if( gamepad2.dpad_left && clawInUse >=0) {
            clawInUse -= 1;
            sleep(200);
        }
        if(gamepad2.dpad_right && clawInUse <=0) {
            clawInUse +=1;
            sleep(200);
        }


        // Effect of Pixel level on shoulder

        if (PixelLevel == 1) {
            ShoulderLevel=260;
            wristlevel = 0.3;

        }
        if (PixelLevel == 2) {
            ShoulderLevel=290;
            wristlevel = 0.35;
        }
        if (PixelLevel == 3) {
            ShoulderLevel=330;
            wristlevel = 0.4;
        }



        if (gamepad2.y) {

            shoulder.setTargetPosition(ShoulderLevel);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(1);
            wrist.setPosition(wristlevel);  

        }
        else if (gamepad2.a) {
            shoulder.setTargetPosition(shoulderFixer+0);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(.3);
            wrist.setPosition(0.25);

            if(shoulder_Position <= 50)
            {
                shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

    }
    private void AirplaneModule () {

        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            air.setPosition(1);
        }

    }
    private void Telemetry () {
        telemetry.addData("Shoulder Fixer:", distance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Speed Movement Multiplier", Speed_Movement_Multiplier);
        telemetry.addData("Horizontal", Lateral);
        telemetry.addData("Axial", Axial);
        telemetry.addData("Gyro Degrees", Gyro_Degrees);
        telemetry.addData("Gyro Radians", Gyro_Radians);
        telemetry.addData("Temp", temp);
        telemetry.addData("Yaw", Orientation2.getYaw(AngleUnit.DEGREES));
        telemetry.addData("shoulder", shoulder.getCurrentPosition());
        telemetry.addData("shoulderStick", shoulderStick);
        telemetry.addData("shoulder_Position", shoulder_Position);
        telemetry.addData("Pixel Level: ", PixelLevel);
        telemetry.addData("Shoulder Level:", ShoulderLevel);
        telemetry.addData("Shoulder Fixer:", shoulderFixer);
        //telemetry.addData("shoulder", shoulder.getTargetPosition());
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
        air.setDirection(Servo.Direction.REVERSE);
        // shoulder Encoder

        shoulder_Position = 0;
        shoulderStick = 0;
        IMU();

        while (opModeIsActive()) {

            // Gamepad2 Variables
            shoulderStick = -gamepad2.left_stick_y*60;
            shoulder_Position = shoulder.getCurrentPosition();

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
            leftFront.setPower((Axial - Lateral + Yaw) * Speed_Movement_Multiplier*FlSpeed);
            rightFront.setPower((Axial + Lateral - Yaw) * Speed_Movement_Multiplier*FrSpeed);
            leftBack.setPower((Axial + Lateral + Yaw) * Speed_Movement_Multiplier*BlSpeed);
            rightBack.setPower((Axial - Lateral - Yaw) * Speed_Movement_Multiplier*BrSpeed);

            // Telemetry
            Telemetry();

            // Gamepad1 COMPLETE
            Movement();

            // Shoulder and Claw READY FOR TEST
            Attachment();

            //Airplane COMPLETE
            AirplaneModule();

        }
    }


}
