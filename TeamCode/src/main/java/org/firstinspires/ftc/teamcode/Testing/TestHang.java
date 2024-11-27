package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechanisms.Slides;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TestHang extends LinearOpMode {

    private Slides slides;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {


        slides = new Slides(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        waitForStart();


        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.y && !previousGamepad1.y) {
                runningActions.add(slides.slideHang());
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                runningActions.add(slides.slideTopBasket());
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                slides.slidesLeftMotor.setPower(1);
                slides.slidesRightMotor.setPower(1);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                slides.slidesLeftMotor.setPower(0);
                slides.slidesRightMotor.setPower(0);
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
        }

    }
}