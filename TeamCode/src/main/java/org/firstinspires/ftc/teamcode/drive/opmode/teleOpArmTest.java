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


@TeleOp

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

           //LeftDrive.setPower(leftPower);
            //RightDrive.setPower(rightPower);

            //MECHANISM CODE





            //double IntakePower = (gamepad1.right_trigger>0.5)?1:((gamepad1.left_trigger>0.5)?-1:0);

            //double SpinnerPower = gamepad1.dpad_left?1:(gamepad1.dpad_right?0:0.5);

            if (gamepad1.a) {
                ArmMotor.setPower(0); //On the ground for starting and intaking
            }
            else if (gamepad1.x) {
                ArmMotor.setPower(-1); //Low level on the goal
            }
            else if (gamepad1.y) {
                ArmMotor.setPower(1); //Mid level on the goal
            }


            //stuff for arm position control
            //ArmMotor.setPower(1);

           //
            //
            //IntakeMotor.setPower(IntakePower);

            //DuckSpinner.setPosition(SpinnerPower);

           // telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
          //  telemetry.update();
        }
    }
}
