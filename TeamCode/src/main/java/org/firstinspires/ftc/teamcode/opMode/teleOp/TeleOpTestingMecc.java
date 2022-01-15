package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp(name="TeleOpTestingMecc",group="TeleOp")
public class TeleOpTestingMecc extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private ScoringArm scoringArm = new ScoringArm();
    public Lift lift = new Lift();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        scoringArm.init(hardwareMap);
        lift.init(hardwareMap);
        lift.setTargetPosition(0.0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        while(opModeIsActive()){
            Pose2d controls = new Pose2d(
                    //Going to test if maybe negative)
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if(!drive.isBusy()) drive.setWeightedDrivePower(controls);
            //-----------INTAKE---------//

            acquirer.run(gamepad1.left_trigger > 0.3, gamepad1.right_trigger > 0.3);

            //-----------CAROUSEL---------//
            carousel.run(gamepad1.left_bumper, gamepad1.right_bumper);

            //-----------SCORING ARM---------//
            if(gamepad1.x){
                //RESET SCORING ARM
                scoringArm.goToStart();
                scoringArm.depositReset();
            } else if (gamepad1.y){
                //DUMP
                scoringArm.dump();
            } else if (gamepad1.b){
                //TUCK POSITION
//                scoringArm.goTo(scoringArm.armMidPos);
                scoringArm.tuck();
            }
            if(gamepad1.a){
                scoringArm.depositReset();
            }
            //-----------LIFT---------//
            if(gamepad1.dpad_left){
                //HOME LIFT
                lift.retracting(true);
                lift.setTargetPosition(lift.minPos);

            } else if (gamepad1.dpad_right){

                // extend to high goal position
                lift.retracting(false);
                lift.setTargetPosition(lift.midPos);
                scoringArm.tuck();

            } else if (gamepad1.dpad_up){
                //increment extension
                lift.retracting(false);
                lift.extend();
            } else if (gamepad1.dpad_down){
                //increment retracting
//                lift.retracting(true);
                lift.retract();

            }
            lift.update();

            //These are the RATIO positions of the servos
            telemetry.addData("Arm homed status:",scoringArm.homed());
            telemetry.addData("Lift target position", lift.getTargetPosition());
            telemetry.addData("Lift retract status:", lift.getRetract());
//            telemetry.addData("kF: ",lift.kF);
//            telemetry.addData("kP: ", lift.coeffs.kP);
//            telemetry.addData("liftL: ", lift.liftLeft.getCurrentPosition());
//            telemetry.addData("liftR: ", lift.liftRight.getCurrentPosition());
            telemetry.update();
        }
    }

}