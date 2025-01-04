package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class TestColorSensor extends LinearOpMode {
    public static double yellowRedb = 0.1;
    public static double yellowRedt = 0.35;
    public static double yellowGreenC = 0.45;
    public static double yellowGreenb = 0.2;
    public static double yellowBluet = 0.42;
    public static double redRedb = 0.1;
    public static double redGreent = 0.4;
    public static double redBluet = 0.3;
    public static double blueRedt = 0.3;
    public static double blueGreent = 0.4;
    public static double blueBlueb = 0.1;
    public static float gain = 50;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private Telemetry tele = dash.getTelemetry();

    @Override
    public void runOpMode() {

        NormalizedColorSensor colorSensor;

        Extendo extendo = new Extendo(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);


        final float[] hsvValues = new float[3];

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            colorSensor.setGain(gain);

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            if (colors.red > yellowRedb && colors.green > yellowGreenb && colors.blue < yellowBluet && (colors.green > yellowGreenC || colors.red < yellowRedt)) {
                telemetry.addData("color","yellow");
                tele.addData("color","yellow");
            } else if (colors.red > redRedb && colors.green < redGreent && colors.blue < redBluet) {
                telemetry.addData("color","red");
                tele.addData("color","red");
            } else if (colors.red < blueRedt && colors.green < blueGreent && colors.blue > blueBlueb) {
                telemetry.addData("color","blue");
                tele.addData("color","blue");
            } else {
                telemetry.addData("color","none");
                tele.addData("color","none");
            }

            if (gamepad1.a) {
                runningActions.add(new SequentialAction(
                        extendo.extend(),
                        intake.flip(),
                        intake.intake()
                ));
            }

            if (gamepad1.b) {
                runningActions.add(new SequentialAction(
                        extendo.retract(),
                        intake.flop(),
                        intake.off()
                ));
            }

            if (gamepad1.x) {
                runningActions.add(intake.off());
            }

            if (gamepad1.y) {
                runningActions.add(intake.intake());
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

            telemetry.addData("redv", colors.red);
            telemetry.addData("bluev", colors.blue);
            telemetry.addData("greenv", colors.green);
            telemetry.update();

            tele.addData("redv", colors.red);
            tele.addData("bluev", colors.blue);
            tele.addData("greenv", colors.green);
            tele.update();


        }
    }

}