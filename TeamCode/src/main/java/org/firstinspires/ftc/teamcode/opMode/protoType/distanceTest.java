package org.firstinspires.ftc.teamcode.opMode.protoType;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.MeccRobot;
import org.firstinspires.ftc.teamcode.hardware.SenseHub;

@TeleOp
public class distanceTest extends LinearOpMode {
    DistanceSensor distance;
    MeccRobot robot = new MeccRobot();
    SenseHub sense = new SenseHub();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.init(hardwareMap,telemetry);
        sense.init(hardwareMap);
        // Get the distance sensor and motor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "sens");
        Rev2mDistanceSensor sens = (Rev2mDistanceSensor)distance;
        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance", String.format("%.01f cm", sens.getDistance(DistanceUnit.CM)));
            telemetry.addData("distance0", String.format("%.01f cm", sense.distance()));
            telemetry.addData("distance0", String.format("inrange", sense.inRange()));


            telemetry.update();

            //robot.run(gamepad1);
        }
    }
}
