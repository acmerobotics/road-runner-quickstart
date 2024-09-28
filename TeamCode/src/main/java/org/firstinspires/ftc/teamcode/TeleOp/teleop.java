package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Arm;
import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Constants;

@TeleOp(name="Driver Teleop")
public class teleop extends LinearOpMode {

    static Robot robot = new Robot();
    static MecanumDrive drive;
    static GamepadEx gamepadone;
    static GamepadEx gamepadtwo;
    public MotorEx Arm1, Arm2;

    public double factor = 3;
    public double turnFactor = 0.7;
    public double multiplier = .5;
    public double armSpeedUp = .6;
    public double armSpeedDown = .5;
    private Arm arm;
    private Claw claw;
    public double strafeFactor = 3;

    public void PIDTEST(int target){
        double Kp = .002;
        double Ki = .02;
        double Kd = .05;

        double reference = .05;

        double integralSum = 0;

        double lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (Arm2.getCurrentPosition() != target) {


            // obtain the encoder position
            int encoderPosition = Arm2.getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum += (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            Arm2.set(-out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);
        drive = new MecanumDrive(robot.TopLeft, robot.TopRight, robot.BottomLeft, robot.BottomRight);
        gamepadone = new GamepadEx(gamepad1);
        gamepadtwo = new GamepadEx(gamepad2);

        robot.Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm = new Arm(robot.Arm2);
        robot.Drone.setPosition(1);

        claw = new Claw(robot.Claw, robot.Claw2);


        //initilize the claw to a closed position
        robot.Hanging.setPosition(0);
//        robot.Claw.setPosition(Constants.ClawOpen);
//        robot.Claw2.setPosition(Constants.Claw2Open);
        claw.ClawClosed();
        robot.YServo.setPosition(Constants.YServoUp);

        waitForStart();
        while (opModeIsActive()) {
//            if(robot.Arm1.getCurrentPosition() >= 950)
//            else multiplier = 1;
//            telemetry.addData("", multiplier);

            drive.driveRobotCentric(
                    -strafeFactor * gamepadone.getLeftX(),
                    -factor * gamepadone.getLeftY(),
                    -turnFactor * gamepadone.getRightX(),
                    true
            );
            if (robot.Arm2.getCurrentPosition() > 1100){
                strafeFactor = 0.75;
            } else {
                strafeFactor = 3;
            }

            if(gamepad2.left_bumper){ //GET INTO POSITION!
                robot.Arm1.setTargetPosition(-Constants.ArmUpTicks);
                robot.Arm2.setTargetPosition(Constants.ArmUpTicks);
                robot.Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.Arm1.setPower(armSpeedUp);
                robot.Arm2.setPower(armSpeedUp);
                while (!isStopRequested() && !(gamepad2.right_bumper)) {
                    telemetry.addLine("" + robot.Arm1.getCurrentPosition() + ", " + robot.Arm2.getCurrentPosition());
                    out_loop();
                    telemetry.update();
                }
            }
            out_loop();
            telemetry.update();
        }
    }

    public void out_loop() throws InterruptedException {

        if(gamepad1.dpad_down){
            robot.YServo.setPosition(Math.max(robot.YServo.getPosition() - 0.002, 0));
            telemetry.addData("Servo Pos: ", robot.YServo.getPosition());
//            telemetry.update();
        }

        if(gamepad1.dpad_up){
            robot.YServo.setPosition(Math.min(robot.YServo.getPosition() + 0.002, 1));
            telemetry.addData("Servo Pos: ", robot.YServo.getPosition());
//            telemetry.update();
        }

        if (gamepad2.left_trigger != 0){
            int current = robot.Arm2.getCurrentPosition();
            int dx = 70;
            robot.Arm2.setTargetPosition(current + dx);
            robot.Arm1.setTargetPosition(-(current + dx));
        }

        if (gamepad2.right_trigger != 0){
            int current = robot.Arm2.getCurrentPosition();
            int dx = -80;
            robot.Arm2.setTargetPosition(current + dx);
            robot.Arm1.setTargetPosition(-(current + dx));
        }

        drive.driveRobotCentric(
                -strafeFactor * gamepadone.getLeftX(),
                -factor * gamepadone.getLeftY(),
                -turnFactor * gamepadone.getRightX(),
                true
        );

        if(gamepad2.y){robot.YServo.setPosition(Constants.YServoDown);}

        if(gamepad2.a){robot.YServo.setPosition(Constants.YServoUp);}
        if(gamepad2.right_stick_button){robot.YServo.setPosition(Constants.YServoDownFull);}

        if(gamepad2.x){ // open claw
            //Claw Open
//            robot.Claw.setPosition(Constants.ClawOpen);
//            robot.Claw2.setPosition(Constants.Claw2Open);
            claw.ClawOpen();
        } else if(gamepad2.dpad_left) { // open claw
//            robot.Claw.setPosition(Constants.ClawOpen);
            claw.Claw1Open();
        } else if(gamepad2.dpad_right) { // open claw

//            robot.Claw2.setPosition(Constants.Claw2Open);
            claw.Claw2Open();
        } else{
            claw.ClawClosed();
            //claw is automatically closed...
//            robot.Claw.setPosition(Constants.ClawClosed);
//            robot.Claw2.setPosition(Constants.Claw2Closed);
        }
        if(gamepad2.b){
            robot.Hanging.setPosition(.38);
            telemetry.addLine("gamepad2 hanging");
//            telemetry.update();
        }
        if(gamepad2.dpad_down){
            telemetry.addLine("dpad down hanging");
//            telemetry.update();
            robot.Hanging.setPosition(0);
        }
        if(gamepad2.dpad_up){
            telemetry.addLine("dpad up hanging");
//            telemetry.update();
            robot.Hanging.setPosition(1);
        }
        if(gamepad1.left_stick_button){
            //lets do right trigger but WAY QUICKER
            int current = robot.Arm1.getCurrentPosition();
            int dx = -110;
            robot.Arm2.setTargetPosition(current + dx);
            robot.Arm1.setTargetPosition(-(current + dx));
        }

        if(gamepad1.left_bumper){
            factor = 0.7;
            turnFactor = 0.5;
        } else{
            factor = 3;
            turnFactor = 0.7;
        }

        if(gamepad1.right_bumper){
            robot.Hanging.setPosition(0);
        }
        if (gamepad2.right_bumper) { //go to zero position. for EVERYTHING
            arm.ArmDown(armSpeedDown);
//            robot.Arm2.setTargetPosition(0);
//            robot.Arm1.setTargetPosition(0);
//            robot.Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.Arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.Arm1.setPower(armSpeedDown);
//            robot.Arm2.setPower(armSpeedDown);
        }

        if (gamepad1.b){
            robot.Drone.setPosition(0);
        }
        if (robot.Arm2.getCurrentPosition() > 1100){
            strafeFactor = 0.75;
        } else {
            strafeFactor = 3;
        }
//        telemetry.update();
    }
}
