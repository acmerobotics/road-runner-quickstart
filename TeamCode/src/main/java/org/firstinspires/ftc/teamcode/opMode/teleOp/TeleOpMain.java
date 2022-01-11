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

@Config
@TeleOp(name="TeleOpMain",group="TeleOp")
public class TeleOpMain extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private ScoringArm scoringArm = new ScoringArm();
    public Lift lift = new Lift();
    public static double height = 5.0;
    @Override
    public void runOpMode() throws InterruptedException{

        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        scoringArm.init(hardwareMap);
        lift.init(hardwareMap);
        lift.targetPosition = 0.0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        while(opModeIsActive()){
            Pose2d controls = new Pose2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if(!drive.isBusy()) drive.setWeightedDrivePower(controls);
            acquirer.run(gamepad1.left_trigger > 0.3, gamepad1.right_trigger > 0.3);
            carousel.run(gamepad1.right_bumper);
//            scoringArm.run(gamepad1.y);
//            scoringArm.deposit(gamepad1.a);
            if(gamepad1.x){
                scoringArm.goToStart();
            } else if (gamepad1.y){
                scoringArm.goToEnd();
            } else if (gamepad1.b){
//                scoringArm.goTo(scoringArm.armMidPos);
                scoringArm.tuck();
            }
            if(gamepad1.a){
                scoringArm.depositReset();
            }
            //LIFT CONTROLS
            if(gamepad1.dpad_left){
                //units in inches
                //HOME LIFT
                telemetry.addData("Homing LIFT:", "yes");
                lift.retracting(true);
                lift.targetPosition = 0.0;

            } else if (gamepad1.dpad_right){
                lift.retracting(false);
                lift.targetPosition = height;
                telemetry.addData("Extending LIFT:", "yes");
                //HIGH GOAL EXTENSION LIFT
            } else if (gamepad1.dpad_up){
                lift.targetPosition+=0.1;
                //Increment up
            } else if (gamepad1.dpad_down){
                lift.targetPosition-=0.1;
                //Increment down
            }
            lift.update();

            //These are the RATIO positions of the servos
            telemetry.addData("Arm homed status:",scoringArm.homed());
            telemetry.addData("Deposit Ratio Position",scoringArm.getPosDeposit());
            telemetry.addData("PivotArm Ratio Position",scoringArm.getPosPivotArm());
            telemetry.addData("Lift home status", lift.targetPosition);
            telemetry.addData("Retracting:", lift.getRetract());
            telemetry.addData("kF: ",lift.kF);
            telemetry.addData("kP: ", lift.coeffs.kP);
            telemetry.update();
        }
    }

}