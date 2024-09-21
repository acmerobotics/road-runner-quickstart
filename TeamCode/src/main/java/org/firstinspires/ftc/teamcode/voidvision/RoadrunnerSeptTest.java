package org.firstinspires.ftc.teamcode.voidvision;


import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RoadrunnerSeptTest", group = "Autonomous")
public class RoadrunnerSeptTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-60, -60, 0), Math.PI / 2)
                .waitSeconds(3)
                .build();


        while (!isStopRequested() && !opModeIsActive()) {
            //int position = visionOutputPosition;
            //telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        //int startPosition = visionOutputPosition;
        //telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1

                        //,trajectoryActionCloseOut
                )
        );
    }
}