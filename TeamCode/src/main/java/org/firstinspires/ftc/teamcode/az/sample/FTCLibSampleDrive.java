package org.firstinspires.ftc.teamcode.az.sample;


import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Localizer;

@TeleOp
public class FTCLibSampleDrive extends LinearOpMode {

    private Motor leftFront, rightFront, leftBack, rightBack, slides, arm;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {

            VoltageSensor voltageSensor = null;
            LazyImu lazyImu = null;
            Localizer localizer = null;
            drive = new MecanumDrive(leftFront, rightBack, leftBack, rightBack, null, null, null);
            driverOp = new GamepadEx(gamepad1);
            slides = new Motor(hardwareMap, "slides");
            arm = new Motor(hardwareMap, "arm");


            if (gamepad1.x) {
                slides.setRunMode(Motor.RunMode.RawPower);
                sleep(2000);
            }
            //if(gamepad1.a){
            //arm.
            //}

            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightY());


        }
    }
}