package org.firstinspires.ftc.teamcode.testCode;
import org.firstinspires.ftc.teamcode.testCode.encoderTest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tfrec.Detector;




@TeleOp
public class TeleOpdrive extends LinearOpMode {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    Servo gripper;
    private DcMotor arm;
    private Detector tfDetector = null;
    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 537.7; //537.7,,28
    static final double DRIVE_GEAR_REDUCTION = 1; //20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 35 * Math.PI;//109.9
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM; //112/109.9
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;//1.0191*25.4

    //Arm encoder
    private void armControl(double power, double inches) {
        int target;


        if (opModeIsActive()) {
            // Create target positions
            target = arm.getCurrentPosition() + (int) (inches * DRIVE_COUNTS_PER_IN);

            //arm.setDirection(DcMotorSimple.Direction.REVERSE);
            // set target position
            arm.setTargetPosition(target);

            //switch to run to position mode
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //run to position at the desiginated power
            arm.setPower(power);


            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (arm.isBusy())) {
            }

            // set motor power back to 0
            arm.setPower(0);


        }
    }
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
         motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
         motorBackLeft = hardwareMap.dcMotor.get("leftRear");
         motorFrontRight = hardwareMap.dcMotor.get("rightFront");
         motorBackRight = hardwareMap.dcMotor.get("rightRear");
         arm = hardwareMap.dcMotor.get("arm");
       // arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.servo.get("gripper");




        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            //arm
            double leftAxis = -gamepad1.left_stick_y;
            double rightAxis = -gamepad1.right_stick_y;

            //arm Power
            double leftPower = -leftAxis;
            double rightPower = rightAxis;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


            //highest junction
            if (gamepad1.x) {

                gripper.setPosition(90); //clamp 90 degree
                sleep(500);
                arm.setPower(1);



            }

            //middle junction
            if(gamepad1.a){
                gripper.setPosition(90); //clamp 90 degree
                sleep(500);
                armControl(1,30);
            }

            //lower junction
            if(gamepad1.b){
                gripper.setPosition(90); //clamp 90 degree
                sleep(500);
                armControl(1,30);
            }

            //ground


            //go down
            if (gamepad1.y){
                gripper.setPosition(0); //release clamp
                sleep(500);
                arm.setPower(-0.8);

            }



        }
    }
}