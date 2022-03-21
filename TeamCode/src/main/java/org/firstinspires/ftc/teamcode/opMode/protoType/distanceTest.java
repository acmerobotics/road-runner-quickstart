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
import org.firstinspires.ftc.teamcode.hardware.TestDistanceSensor;

@TeleOp (group = "prototype")
public class distanceTest extends LinearOpMode {
    TestDistanceSensor distance = new TestDistanceSensor(this);
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        distance.init(hardwareMap);
        // Get the distance sensor and motor from hardwareMap
        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {

            distance.telemetry(telemetry);

            telemetry.update();

            //robot.run(gamepad1);
        }
    }
}
