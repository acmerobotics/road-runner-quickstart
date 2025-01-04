package org.firstinspires.ftc.teamcode.TestArchive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.SlidesV2;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class speciClaw extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private Telemetry tele = dash.getTelemetry();

    @Override
    public void runOpMode() {

        Claw claw = new Claw(hardwareMap);
        SlidesV2 slides = new SlidesV2(hardwareMap, true);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            slides.updateMotors();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.b && !previousGamepad1.b) {
                runningActions.add(new SequentialAction(
                        slides.slideTopBarClip(),
                        claw.open(),
                        slides.retract()
                ));
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                runningActions.add(slides.slideTopBar());
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                runningActions.add(claw.close());
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                runningActions.add(claw.open());
            }

            slides.changeTarget((int) (currentGamepad1.left_stick_y * 60));


            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);

            telemetry.addData("PID", slides.getPID());
            telemetry.addData("PID target", slides.getTarget());
            telemetry.addData("Slides Left", slides.slidesLeftMotor.getCurrentPosition());
            telemetry.addData("Slides Right", slides.slidesRightMotor.getCurrentPosition());
            telemetry.update();

        }

        }
    }

