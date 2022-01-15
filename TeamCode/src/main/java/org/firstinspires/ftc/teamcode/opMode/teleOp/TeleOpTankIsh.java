package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.LiftAndScoring;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;

@Config
@TeleOp(name="TeleOpIsh",group="TeleOp")
public class TeleOpTankIsh extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private LiftAndScoring scoring = new LiftAndScoring();
    public static double omega_multiplier = 1.6;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException{
        boolean formerY = false;
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        scoring.init(hardwareMap);

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        while(opModeIsActive()){
            Pose2d controls = new Pose2d(
                    //Going to test if maybe negative)
                    gamepad1.left_stick_y,
                    0,
                    gamepad1.right_stick_x * omega_multiplier
            );
            telemetry.addData("rightstickx",gamepad1.right_stick_x);

            if(!drive.isBusy()) drive.setDrivePower(controls);
            //-----------INTAKE---------//

            acquirer.run(gamepad1.left_trigger > 0.3, gamepad1.right_trigger > 0.3);

            //-----------CAROUSEL---------//
            carousel.run(gamepad1.dpad_left,gamepad1.dpad_right);
            //-----------SCORING ARM---------//
            //toggle scoring arm raising

            scoring.toggleSlides(gamepad1.right_bumper);
            scoring.lowGoal(gamepad1.x);
            scoring.toggleDepo(gamepad1.y);

            scoring.update();
            //

            //These are the RATIO positions of the servos

//            telemetry.addData("kF: ",lift.kF);
//            telemetry.addData("kP: ", lift.coeffs.kP);
//            telemetry.addData("liftL: ", lift.liftLeft.getCurrentPosition());
//            telemetry.addData("liftR: ", lift.liftRight.getCurrentPosition());
            telemetry.update();
        }
    }

}