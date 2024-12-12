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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auto.Actions.ArmActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;


import java.util.ArrayList;
import java.util.List;
@TeleOp(name="sigma opMode", group="For Sigmas")
public class TestOpMode extends OpMode {


    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmActions armActions = new ArmActions(hardwareMap);

        TelemetryPacket packet = new TelemetryPacket();
        double SLOW_DOWN_FACTOR = 1;
        double forward = -gamepad1.left_stick_y * SLOW_DOWN_FACTOR;
        double right = -gamepad1.left_stick_x * SLOW_DOWN_FACTOR;
        double rotate = gamepad1.right_stick_x * SLOW_DOWN_FACTOR;
        double robotAngle = -drive.getRoboticOrientation();
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        telemetry.addData("Running TeleOp for:", "15344");
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        newForward,
                        newRight
                ),
                rotate
        ));

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
        if (gamepad1.a) {
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    new ParallelAction(
                    new InstantAction(() -> armActions.rightSlide.setPower(1)),
                    new InstantAction(() ->  armActions.leftSlide.setPower(1))
                    )
            ));
            packet.put("runningactions", runningActions);
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

    }
}
