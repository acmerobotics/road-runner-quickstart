package org.firstinspires.ftc.teamcode.unknownFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@SuppressWarnings("unused")
@Config
@TeleOp
public class ArrayTestOpMode extends LinearOpMode {
    public static class NestedClass {
        public int a, b;
    }

    public static int[] array = new int[3];
    public static NestedClass[] innerArray =
        new NestedClass[] {new NestedClass(), new NestedClass(), new NestedClass()};

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        array = new int[2];
        while (opModeIsActive()) {
            telemetry.addData("array0", array[0]);
            telemetry.addData("array1", array[1]);
            telemetry.addData("innerArray0A", innerArray[0].a);
            telemetry.addData("innerArray1B", innerArray[1].b);
            telemetry.addData("innerArray2A", innerArray[2].a);
            telemetry.update();
        }
    }
}
