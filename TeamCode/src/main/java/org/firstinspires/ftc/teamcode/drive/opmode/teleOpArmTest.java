package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Arm Test", group = "Test Code")


public class teleOpArmTest extends LinearOpMode {
    private Blinker Control_Hub;
    private Gyroscope imu;
    private DcMotor LeftDrive;
    private DcMotor RightDrive;
    private DcMotor ArmMotor;
    private DcMotor IntakeMotor;
    private Servo DuckSpinner;


    @Override
    public void runOpMode() {
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        //RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        ArmMotor = hardwareMap.get(DcMotor.class, "arm");
        //ArmMotor.setTargetPosition(0);
        //ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       // IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");

      //  DuckSpinner = hardwareMap.get(Servo.class, "DuckSpinner");

        //int ArmTarget = 0;

       // telemetry.addData("Status", "Initialized");
       // telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //DRIVETRAIN CODE
            double leftAxis = -gamepad1.left_stick_y;
            double rightAxis = -gamepad1.right_stick_y;

            double leftPower = -leftAxis;
            double rightPower = rightAxis;


            if (gamepad1.a) {
                ArmMotor.setPower(0);
            }
            else if (gamepad1.x) {
                ArmMotor.setPower(-0.5);
            }
            else if (gamepad1.y) {
                ArmMotor.setPower(0.5);
            }

        }
    }
}
