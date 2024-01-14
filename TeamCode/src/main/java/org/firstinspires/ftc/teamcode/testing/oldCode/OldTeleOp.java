package org.firstinspires.ftc.teamcode.testing.oldCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="OldTeleOp", group="Linear Opmode")
@Config
public class OldTeleOp extends LinearOpMode
{
    public static int base = 10;
    public static double power = .5;
    public static int offsetNumSlide = 50;
    public static double offsetNumServo;
    public static int sleepTime = 500;
    public static int sleepTimeClaw = 1000;

    //Slide
    public static int desiredPos;
    public static int slideFirstUp = 220;
    public static int slideSecondUp = 1000;
    public static int slideFirstDown = 800;
    public static int slideZeroPos = 75;

    //Elbow Servo
    public static double desiredPosElServ;
    public static double elservoFirstUp = .455;
    public static double elServoZeroPos = .4955;

    //Claw Servo
    public static double desiredPosClawServ;
    public static double clawCatchPos = .125;
    public static double clawDropPos = 0.1;
    public static double clawClosePos = 0.04;
    public static int catchDrop = 30;

    public static boolean yPress = false;
    public static boolean up = false;

    //SPACE BETWEEN ARM CODE AND DRIVETRAIN CODE_______________________________________________________________
    //_________________________________________________________________________________________________________
    //_________________________________________________________________________________________________________

    public static double powerModifier = 1;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    /*

    private VisionPortal visionPortal;  // The variable to store our instance of the vision portal.
    private AprilTagProcessor aprilTag; // The variable to store our instance of the AprilTag processor.
    IMU imu;
    boolean fieldCentric = true;

     */

    @Override
    public void runOpMode()
    {
        DcMotor viperSlideLeft = hardwareMap.get(DcMotor.class, "viper_slide_left");
        viperSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor viperSlideRight = hardwareMap.get(DcMotor.class, "viper_slide_right");
        viperSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        Servo elbow_servo = hardwareMap.get(Servo.class, "elbow_servo" );
        Servo claw_servo = hardwareMap.get(Servo.class, "claw_servo" );

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base = viperSlideLeft.getCurrentPosition();
        viperSlideLeft.setTargetPosition(slideZeroPos + base);
        viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideLeft.setPower(power);

        elbow_servo.setPosition(elServoZeroPos);

        claw_servo.setPosition(clawClosePos);

        //SPACE BETWEEN ARM CODE AND DRIVETRAIN CODE_______________________________________________________________
        //_________________________________________________________________________________________________________
        //_________________________________________________________________________________________________________

        //imu = hardwareMap.get(IMU.class, "imu");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        front_left  = hardwareMap.get(DcMotor.class, "leftfront_drive");
        back_left  = hardwareMap.get(DcMotor.class, "leftback_drive");
        front_right = hardwareMap.get(DcMotor.class, "rightfront_drive");
        back_right = hardwareMap.get(DcMotor.class, "rightback_drive");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        /*
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
         */
        //Wait for start part of both
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            if (gamepad1.left_stick_button){
                slideSecondUp = 1300;
            }
            if (gamepad1.right_stick_button){
                slideSecondUp = 1000;
            }
            if (gamepad1.a && up == false) {
                desiredPos = slideFirstUp + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide) {
                    telemetry(viperSlideLeft.getCurrentPosition(), desiredPos, elbow_servo.getPosition(), claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);
                }

                desiredPosElServ = elservoFirstUp;
                elbow_servo.setPosition(desiredPosElServ);
                sleep(sleepTime);
                /*while (desiredPosElServ < elbow_servo.getPosition() - offsetNumServo || desiredPosElServ > elbow_servo.getPosition() + offsetNumServo)
                {
                    telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ);
                }*/

                desiredPos = slideSecondUp + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide) {
                    telemetry(viperSlideLeft.getCurrentPosition(), desiredPos, elbow_servo.getPosition(), claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);
                }
                up = true;
            }
            if (gamepad1.x) {
                claw_servo.setPosition(clawDropPos);
                sleep(sleepTimeClaw);
                claw_servo.setPosition(clawClosePos);
            }
            if (gamepad1.y && yPress == false) {
                desiredPos = catchDrop + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide) {
                    telemetry(viperSlideLeft.getCurrentPosition(), desiredPos, elbow_servo.getPosition(), claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);
                }
                claw_servo.setPosition(clawCatchPos);
                sleep(200);
                yPress = true;
            } else if (gamepad1.y && yPress == true) {
                claw_servo.setPosition(clawClosePos);
                sleep(550);
                desiredPos = slideZeroPos + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide) {
                    telemetry(viperSlideLeft.getCurrentPosition(), desiredPos, elbow_servo.getPosition(), claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);
                }
                sleep(200);
                yPress = false;
            }
            if (gamepad1.b && up == true) {
                desiredPos = slideFirstDown + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide) {
                    telemetry(viperSlideLeft.getCurrentPosition(), desiredPos, elbow_servo.getPosition(), claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);
                }

                desiredPosElServ = elServoZeroPos;
                elbow_servo.setPosition(desiredPosElServ);
                sleep(sleepTime);
                /*while (desiredPosElServ < elbow_servo.getPosition() - offsetNumServo || desiredPosElServ > elbow_servo.getPosition() + offsetNumServo)
                {
                    telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ);
                }*/

                desiredPos = slideZeroPos + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide) {
                    telemetry(viperSlideLeft.getCurrentPosition(), desiredPos, elbow_servo.getPosition(), claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);
                }
                up = false;

            }
            /*if (gamepad1.left_bumper){
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                viperSlideLeft.setPower(gamepad1.left_stick_y);*/

            if (gamepad1.dpad_right) {
                viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                viperSlideLeft.setTargetPosition(slideZeroPos + base);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);

                elbow_servo.setPosition(elServoZeroPos);

                claw_servo.setPosition(clawClosePos);
            }
            if (gamepad1.dpad_down) {
                base = base - 20;
                viperSlideLeft.setTargetPosition(slideZeroPos + base);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                sleep(750);
            }
            if (gamepad1.dpad_up) {
                base = base + 20;
                viperSlideLeft.setTargetPosition(slideZeroPos + base);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                sleep(750);
            }
            if (gamepad1.dpad_left) {
                if (up == true) {
                    up = false;
                } else if (up == false) {
                    up = true;
                }
            }
            telemetry(viperSlideLeft.getCurrentPosition(), desiredPos, elbow_servo.getPosition(), claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);

            //SPACE BETWEEN ARM CODE AND DRIVETRAIN CODE_______________________________________________________________
            //_________________________________________________________________________________________________________
            //_________________________________________________________________________________________________________

            if (gamepad1.left_bumper) {
                powerModifier = .5;
            }
            if (gamepad1.right_bumper) {
                powerModifier = 1;
            }


            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            front_left.setPower(powerModifier * leftFrontPower);
            front_right.setPower(powerModifier * rightFrontPower);
            back_left.setPower(powerModifier * leftBackPower);
            back_right.setPower(powerModifier * rightBackPower);


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            /*
            double powerReduction = .6;
            double y   = -gamepad1.left_stick_y * powerReduction;  // Note: pushing stick forward gives negative value
            double x =  gamepad1.left_stick_x * powerReduction * 1.1; // Correct strafing
            double yaw     =  gamepad1.right_stick_x * powerReduction;

            if (fieldCentric)
            {
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                moveRobot(rotY, rotX, yaw);
            }
            else
            {
                moveRobot(y, x, yaw);
            }
            // Check to see if heading reset is requested
            if (gamepad1.right_bumper) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            }
            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            */
        }
    }


    private void telemetry(int motorPosL, int desiredposition, double elservpos, double clawservpos, double desiredPosElServ, double desiredPosClawServ, int base)
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("Left Slide Motor Position", motorPosL);
        dashboardTelemetry.addData("Elbow Servo Position", elservpos);
        dashboardTelemetry.addData("Claw Servo Position", clawservpos);

        dashboardTelemetry.addData("Desired Position", desiredposition);
        dashboardTelemetry.addData("Elbow Servo Desired Position", desiredPosElServ);
        dashboardTelemetry.addData("Claw Servo Desired Position", desiredPosClawServ);

        dashboardTelemetry.addData("Current Base", base);



        telemetry.addData("Left Slide Motor Position", motorPosL);
        telemetry.addData("Elbow Servo Position", elservpos);
        telemetry.addData("Claw Servo Position", clawservpos);

        telemetry.addData("Desired Position", desiredposition);
        telemetry.addData("Elbow Servo Desired Position", desiredPosElServ);
        telemetry.addData("Claw Servo Desired Position", desiredPosClawServ);

        telemetry.addData("Current Base", base);

        dashboardTelemetry.update();
        telemetry.update();
    }

    //SPACE BETWEEN ARM CODE AND DRIVETRAIN CODE_______________________________________________________________
    //_________________________________________________________________________________________________________
    //_________________________________________________________________________________________________________
    /*
    private void moveRobot(double axial, double lateral, double yaw) {
        double max;


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad


        // Send calculated power to wheels
        front_left.setPower(leftFrontPower);
        front_right.setPower(rightFrontPower);
        back_left.setPower(leftBackPower);
        back_right.setPower(rightBackPower);

    }

     */

}