package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class LimelightAngleCorrection extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        waitForStart();
        limelight.start();

        LLResult result = limelight.getLatestResult();
        double tx = result.getTx();
        double ty = result.getTy();

        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.update();

        if (tx >= 0) {
            drive.setPowers(0.1, -0.1, 0.1, -0.1);
            telemetry.addLine("Turning Right");
        }
        else if (tx <= 0) {
            drive.setPowers(-0.1, 0.1, -0.1, 0.1);
            telemetry.addLine("Tuning Left");
        }
        else {
            drive.setPowers(0,0,0,0);
            telemetry.addLine("Doing Nothing");
        }
        telemetry.update();
    }
}
