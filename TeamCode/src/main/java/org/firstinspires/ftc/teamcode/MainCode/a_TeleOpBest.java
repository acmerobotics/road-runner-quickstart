package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="a_TeleOpBest", group="Linear Opmode")
@Config
public class a_TeleOpBest extends LinearOpMode
{
    public static int base;
    public static double power = .5;
    public static int offsetNumSlide = 50;
    public static double offsetNumServo;
    public static int sleepTime = 500;
    public static int sleepTimeClaw = 1500;

    //Slide
    public static int desiredPos;
    public static int slideFirstUp = 200;
    public static int slideSecondUp = 1700;
    public static int slideFirstDown = 800;
    public static int slideZeroPos = 50;

    //Elbow Servo
    public static double desiredPosElServ;
    public static double elservoFirstUp = .45;
    public static double elServoZeroPos = .4955;

    //Claw Servo
    public static double desiredPosClawServ;
    public static double clawCatchPos = .19;
    public static double clawDropPos = 0.1;
    public static double clawClosePos = 0.04;
    public static int catchDrop = 30;

    boolean yPress = false;
    boolean up = false;

    //SPACE BETWEEN ARM CODE AND DRIVETRAIN CODE_______________________________________________________________
    //_________________________________________________________________________________________________________
    //_________________________________________________________________________________________________________

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

        waitForStart();
        while(opModeIsActive())
        {
            if (gamepad1.a && up == false)
            {
                desiredPos = slideFirstUp + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide)
                {
                    telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);                }

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
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide)
                {
                    telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);                }
                up = true;
            }
            if (gamepad1.x){
                claw_servo.setPosition(clawDropPos);
                sleep(sleepTimeClaw);
                claw_servo.setPosition(clawClosePos);
            }
            if (gamepad1.y && yPress == false)
            {
                desiredPos = catchDrop + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide)
                {
                    telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);                }
                claw_servo.setPosition(clawCatchPos);
                sleep(200);
                yPress = true;
            }
            else if (gamepad1.y && yPress == true)
            {
                desiredPos = slideZeroPos + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide)
                {
                    telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);                }
                claw_servo.setPosition(clawClosePos);
                sleep(200);
                yPress = false;
            }
            if (gamepad1.b && up == true)
            {
                desiredPos = slideFirstDown + base;
                viperSlideLeft.setTargetPosition(desiredPos);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide)
                {
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
                while (desiredPos < viperSlideLeft.getCurrentPosition() - offsetNumSlide || desiredPos > viperSlideLeft.getCurrentPosition() + offsetNumSlide)
                {
                    telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);                }
                up = false;

            }
            if (gamepad1.left_bumper){
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                viperSlideLeft.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.dpad_right){
                viperSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                base = viperSlideLeft.getCurrentPosition();
                viperSlideLeft.setTargetPosition(slideZeroPos + base);
                viperSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlideLeft.setPower(power);

                elbow_servo.setPosition(elServoZeroPos);

                claw_servo.setPosition(clawClosePos);
            }
            if (gamepad1.dpad_down) {
                base = base - 4;
            }
            if (gamepad1.dpad_up){
                base = base + 4;
            }
            if (gamepad1.dpad_left){
                if (up == true){
                    up = false;
                }
                else if (up == false){
                    up = true;
                }
            }
            telemetry(viperSlideLeft.getCurrentPosition(),desiredPos, elbow_servo.getPosition(),claw_servo.getPosition(), desiredPosElServ, desiredPosClawServ, base);

            //SPACE BETWEEN ARM CODE AND DRIVETRAIN CODE_______________________________________________________________
            //_________________________________________________________________________________________________________
            //_________________________________________________________________________________________________________

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

        dashboardTelemetry.update();
        telemetry.update();
    }

}