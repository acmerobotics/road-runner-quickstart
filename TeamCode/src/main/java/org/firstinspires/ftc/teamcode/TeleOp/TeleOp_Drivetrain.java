package org.firstinspires.ftc.teamcode.TeleOp;

//importing the statements for the code below
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="TeleOp Drivetrain", group="Iterative TeamCode")
//@Disabled
public class TeleOp_Drivetrain extends OpMode {

    //defining all of the variables needed for the code
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LFMotor, LBMotor, RFMotor, RBMotor;//, armMotor, armMotor2, clawMotor;
    //private Servo rotateServo, clawServo, foundServo, foundServo2, skystoneServo, skystoneClamp;
    //private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private boolean fieldRelativeMode = false;
    private double globalAngle, speed = 0.5;//hello


    @Override
    public void init() throws IllegalArgumentException{

        //grabbing the hardware from the expansion hubs, and the configuration
        LFMotor  = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotor.class, "RB Motor");
        /*armMotor = hardwareMap.get(DcMotor.class, "Arm Motor 1");
        armMotor2 = hardwareMap.get(DcMotor.class, "Arm Motor 2");
        clawMotor = hardwareMap.get(DcMotor.class,"Claw Up Motor");

        rotateServo = hardwareMap.get(Servo.class, "Rotate Servo");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        foundServo = hardwareMap.get(Servo.class, "found servo");
        foundServo2 = hardwareMap.get(Servo.class, "found servo 2");
        skystoneServo = hardwareMap.get(Servo.class, "Skystone servo");
        skystoneClamp = hardwareMap.get(Servo.class, "Skystone Clamp");

        imu = hardwareMap.get(BNO055IMU.class, "imu");*/

        //reversing the motors that need to be reversed, otherwise it sets it as forward
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        /*armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        clawMotor.setDirection(DcMotor.Direction.FORWARD);

        rotateServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
        foundServo2.setDirection(Servo.Direction.REVERSE);
        foundServo.setDirection(Servo.Direction.FORWARD);
        skystoneServo.setDirection(Servo.Direction.FORWARD);
        skystoneClamp.setDirection(Servo.Direction.FORWARD);*/

        //setting up the IMU on the expansion hubs, for our use
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        //defining the value to get from phones
        double LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;
        float slidesValue;

        //checking to see if field relative mode is on
        /*if (gamepad1.back) {
            resetAngle();
            fieldRelativeMode = !fieldRelativeMode;
        }*/

        telemetry.addData("FieldRelative?", fieldRelativeMode);

        //getting the movement values from the gamepad
        yValue = gamepad1.left_stick_y;
        turnValue = gamepad1.right_stick_x;
        xValue = gamepad1.left_stick_x;

        //changing the values for the field relative mode
        /*if (fieldRelativeMode){
            double angle = getAngle();
            double tempX = (xValue * Math.cos(Math.toRadians(angle))) - (yValue * Math.sin(Math.toRadians(angle)));
            yValue = (xValue * Math.sin(Math.toRadians(angle))) + (yValue * Math.cos(Math.toRadians(angle)));
            xValue = tempX;
        }*/

        //getting the values for the powers for each motor
        LFPower = Range.clip(-yValue + turnValue + xValue,-1,1);
        LBPower = Range.clip(-yValue + turnValue - xValue,-1,1);
        RBPower = Range.clip(-yValue - turnValue + xValue,-1,1);
        RFPower = Range.clip(-yValue - turnValue - xValue,-1,1);

        //applying the ramping up and ramping down features
        if (LFPower < 0){
            LFPower = (float) -Math.pow(Math.abs(LFPower),2);
        } else if (LFPower > 0){
            LFPower = (float) Math.pow(Math.abs(LFPower),2);
        }

        if (LBPower < 0){
            LBPower = (float) -Math.pow(Math.abs(LBPower),2);
        } else if (LBPower > 0){
            LBPower = (float) Math.pow(Math.abs(LBPower),2);
        }

        if (RFPower < 0){
            RFPower = (float) -Math.pow(Math.abs(RFPower),2);
        } else if (RFPower > 0){
            RFPower = (float) Math.pow(Math.abs(RFPower),2);
        }

        if (RBPower < 0){
            RBPower = (float) -Math.pow(Math.abs(RBPower),2);
        } else if (RBPower > 0){
            RBPower = (float) Math.pow(Math.abs(RBPower),2);
        }

        slidesValue = gamepad2.left_stick_y;

        if (gamepad1.a){
            speed = 0.3;
        } else{
            speed = 0.5;
        }

        //setting the powers for each of the motors
        LFMotor.setPower(Range.clip(LFPower, -speed, speed));
        LBMotor.setPower(Range.clip(LBPower, -speed, speed));
        RFMotor.setPower(Range.clip(RFPower, -speed, speed));
        RBMotor.setPower(Range.clip(RBPower, -speed, speed));

        /*//getting the double reverse 4 bar linkage to move up and down
        if (slidesValue == 0){
            clawMotor.setPower(-0.2);
        } else {//if (slidesValue >= 0)
            clawMotor.setPower(Range.clip(slidesValue, -0.7 , -0.02));
        }

        //moving the claw servo to pick up or release the stone
        if (gamepad1.x) {
            clawServo.setPosition(Servo.MAX_POSITION);
        }
        if (gamepad1.y) {
            clawServo.setPosition(((Servo.MAX_POSITION - Servo.MIN_POSITION)/2) + Servo.MIN_POSITION + 0.05);
        }

        //Using the intake, intaking or releasing
        if(gamepad1.right_bumper){
            armMotor.setPower(-0.5);
            armMotor2.setPower(0.5);
        } else if(gamepad1.left_bumper){
            armMotor.setPower(0.5);
            armMotor2.setPower(-0.5);
        } else{
            armMotor.setPower(0);
            armMotor2.setPower(0);
        }

        //moving the claw to move it from the inside of the robot to the outside
        if (gamepad2.right_bumper){
            rotateServo.setPosition(Servo.MIN_POSITION);
        } else if(gamepad2.left_bumper){
            rotateServo.setPosition(Servo.MAX_POSITION);
        }

        //using the foundation servos to pick up the foundation
        if (gamepad2.x) {
            telemetry.addData("x","pressed");
            foundServo.setPosition(0.4);
            foundServo2.setPosition(0.6);
        }
        if (gamepad2.y) {
            telemetry.addData("y","pressed");
            foundServo.setPosition(0.6);
            foundServo2.setPosition(0.8);

        }

        //using the stone claw to pick up the stones
        if (gamepad2.a) {
            telemetry.addData("a","pressed");
            skystoneServo.setPosition(0.527);
            skystoneClamp.setPosition(0.9);
        }
        if (gamepad2.b) {
            telemetry.addData("b", "pressed");
            skystoneClamp.setPosition(0.85);
            skystoneServo.setPosition(0.49);
        }*/
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }


    @Override
    public void stop() {
    }

    //reseting the IMU
    /*private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    //getting the angle from the IMU
    /*private double getAngle()
    {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }*/

}