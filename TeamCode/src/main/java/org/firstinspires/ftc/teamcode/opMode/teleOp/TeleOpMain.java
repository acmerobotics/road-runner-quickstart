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
@TeleOp(name="TeleOpMain",group="TeleOp")
public class TeleOpMain extends LinearOpMode {
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
            acquirer.run(gamepad1.left_trigger > 0.3, gamepad1.right_trigger > 0.3);
            carousel.run(gamepad1.right_bumper);
//            scoringArm.run(gamepad1.y);
//            scoringArm.deposit(gamepad1.a);
            if(gamepad1.x){
                scoringArm.goToStart();
            } else if (gamepad1.y){
                scoringArm.dump();
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
                lift.retracting(true);
                lift.setTargetPosition(lift.minPos);

            } else if (gamepad1.dpad_right){
                lift.retracting(false);
                lift.setTargetPosition(lift.midPos);
                //HIGH GOAL EXTENSION LIFT
            } else if (gamepad1.dpad_up){
                lift.retracting(false);
                lift.extend();
                //Increment up
            } else if (gamepad1.dpad_down){
//                lift.retracting(true);
                lift.retract();
                //Increment down
            }
            lift.update();

            //These are the RATIO positions of the servos
            telemetry.addData("Arm homed status:",scoringArm.homed());
            telemetry.addData("Deposit Ratio Position",scoringArm.getPosDeposit());
            telemetry.addData("PivotArm Ratio Position",scoringArm.getPosPivotArm());
            telemetry.addData("Lift target position", lift.getTargetPosition());
            telemetry.addData("Retracting:", lift.getRetract());
            telemetry.addData("kF: ",lift.kF);
            telemetry.addData("kP: ", lift.coeffs.kP);
            telemetry.addData("liftL: ", lift.liftLeft.getCurrentPosition());
            telemetry.addData("liftR: ", lift.liftRight.getCurrentPosition());
            telemetry.addData("Target position", lift.getTargetPosition());

            telemetry.update();
        }
    }

}