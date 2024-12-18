package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TestTeleRR extends LinearOpMode {

    private boolean driveSwitch = false;

    @Override
    public void runOpMode() {

        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Action basketSub1 = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(15))
                .strafeToLinearHeading(new Vector2d(40, -10), Math.toRadians(15))
                .turnTo(Math.toRadians(-45))
                .build();
        Action basketSub2 = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-15))
                .strafeToLinearHeading(new Vector2d(30, -20), Math.toRadians(15))
                .turnTo(Math.toRadians(45))
                .build();
        Action observationSub1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(30, -20), Math.toRadians(-135))
                .turnTo(Math.toRadians(45))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            double leftx = gamepad1.left_stick_x;
            double lefty = -gamepad1.left_stick_y;
            double rightx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(lefty) + Math.abs(leftx) + Math.abs(rightx), 1);
            double frontLeftPower = (lefty + leftx + rightx) / denominator;
            double backLeftPower = (lefty - leftx + rightx) / denominator;
            double frontRightPower = (lefty - leftx - rightx) / denominator;
            double backRightPower = (lefty + leftx - rightx) / denominator;

            if (gamepad1.a) {
                driveSwitch = !driveSwitch;
            }
            if (driveSwitch) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(leftx, lefty), rightx));
            } else {
                drive.leftFront.setPower(frontLeftPower);
                drive.rightFront.setPower(backLeftPower);
                drive.rightFront.setPower(frontRightPower);
                drive.rightBack.setPower(backRightPower);
            }

            if (gamepad1.options) {
                drive.pose = new Pose2d(0, 0, 0);
            }

            if (gamepad1.x) {
                runningActions.add(basketSub1);
            }
            if (gamepad1.a) {
                runningActions.add(basketSub2);
            }
            if (gamepad1.b) {
                runningActions.add(observationSub1);
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);

            telemetry.addData("leftx", leftx);
            telemetry.addData("lefty", lefty);
            telemetry.addData("rightx", rightx);
            telemetry.update();
        }

    }
}
