package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous
public class LimelightAngleCorrection extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.setMsTransmissionInterval(11);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        limelight.pipelineSwitch(0);

        String targetLabel1 = "blocksYellow";
        String targetLabel2 = "blocksBlue";
        String targetLabel3 = "blocksRed";

        String detectorResults;

        waitForStart();
        limelight.start();

        while (opModeIsActive() && !isStopRequested()) {
            LLResult result = limelight.getLatestResult();
            double tx = result.getTx();
            double ty = result.getTy();

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.update();

            double power = 0.2 * (tx/10);

            telemetry.addData("power", power);

            List<DetectorResult> results = result.getDetectorResults();

            if (tx > 0.5) {
                drive.setPowers(power, -power, power, -power);
                telemetry.addLine("Turning Right");

            } else if (tx < -0.5) {
                drive.setPowers(power, -power, power, -power);
                telemetry.addLine("Turning Left");
            } else {
                drive.setPowers(0, 0, 0, 0);
                telemetry.addLine("Doing Nothing");
            }

            telemetry.addData("Result name", results.get(0).getClassName());
            telemetry.update();

        }

    }
}
