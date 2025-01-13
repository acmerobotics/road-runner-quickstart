package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Teleop.Actions.ArmActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;


import java.util.ArrayList;
import java.util.List;
@TeleOp(name="sigma opMode", group="For Sigmas")
public class TestOpMode extends OpMode {


    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();



    @Override
    public void init() {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor leftSlide;
        DcMotor rightSlide;
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        ArmActions armActions = new ArmActions(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        IMU imu = drive.lazyImu.get();


        TelemetryPacket packet = new TelemetryPacket();
        double SLOW_DOWN_FACTOR = 1;
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        telemetry.addData("Running TeleOp for:", "15344");
        drive.leftBack.setPower(backLeftPower);
        drive.leftFront.setPower(frontLeftPower);
        drive.rightBack.setPower(backRightPower);
        drive.rightFront.setPower(frontRightPower);

        drive.updatePoseEstimate();
        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;


        if (gamepad1.left_bumper) {
            armActions.leftSlide.setPower(-0.5);
            armActions.rightSlide.setPower(-0.5);
        } else if (gamepad1.right_bumper) {
            armActions.leftSlide.setPower(1);
            armActions.rightSlide.setPower(1);
        } else {
            armActions.leftSlide.setPower(0);
            armActions.rightSlide.setPower(0);
        }



        if (gamepad1.a) {
            runningActions.add(
                    armActions.openClaw()
            );
            telemetry.addData("runningactions", runningActions);
        }
        else if (gamepad1.x) {
            runningActions.add(
                    armActions.closeClaw()
            );
            telemetry.addData("runningactions", runningActions);
        }

        runningActions.add(
                armActions.raiseClaw()
        );

        runningActions.add(new InstantAction(() -> armActions.clawPivot.setPosition(0.61)));

        //gamepad 2

        if (gamepad2.left_bumper) {
            runningActions.add(
                    new ParallelAction(
                        new InstantAction(() -> armActions.leftHang.setPower(-1)),
                        new InstantAction(() -> armActions.rightHang.setPower(1))
                    ));
        } else if(gamepad2.right_bumper) {
            runningActions.add(
                    new ParallelAction(
                            new InstantAction(() -> armActions.leftHang.setPower(1)),
                            new InstantAction(() -> armActions.rightHang.setPower(-1))
                    ));
        } else {
            runningActions.add(
                    new ParallelAction(
                            new InstantAction(() -> armActions.leftHang.setPower(0)),
                            new InstantAction(() -> armActions.rightHang.setPower(0))
                    ));
        }


        //Intake Stuff

        if (gamepad2.dpad_left) {
            runningActions.add(armActions.runSlide());
        } else if (gamepad2.dpad_right) {
            runningActions.add(armActions.reverseSlide());
        } else {
            runningActions.add(armActions.stopSlide());
        }

        if(gamepad2.dpad_up){
            runningActions.add(armActions.raiseIntake());
        }

        else if(gamepad2.dpad_down){
            runningActions.add(armActions.lowerIntake());
        }

        if(gamepad2.a){
            runningActions.add(armActions.reverseIntake());
        }
        else if(gamepad2.b){
            runningActions.add(armActions.runIntake());
        }
        else{
            runningActions.add(armActions.stopIntake());
        }

        dash.sendTelemetryPacket(packet);

    }
}
