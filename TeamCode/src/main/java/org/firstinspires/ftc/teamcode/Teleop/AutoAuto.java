package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Dictionary;
import java.util.Hashtable;

@TeleOp
public class AutoAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d StartPose = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        boolean mappingState = false; // Switch to make sure no infinite execution

        Dictionary<String, Double> mappingTable = new Hashtable<>();


        double mapping_x = 0.00; // Place holder for last X pos
        double mapping_y = 0.00; // Place holder for last Y pos
        double mapping_heading = 0.00;

        int count = 0; // Count variable for dict keys, temporary solution? Hopefully better in the future

        waitForStart();

        while (opModeIsActive()){

            double x = drive.pose.position.x; // drive pos x
            double y = drive.pose.position.y; // drive pos y
            double heading = drive.pose.heading.toDouble();

            if (currentGamepad1.a && !mappingState){
                mapping_x = x;
                mapping_y = y;
                mapping_heading = heading;

                String x_key = String.format("x_pos_%s", count);
                String y_key = String.format("y_pos_%s", count);
                String heading_key = String.format("heading_%s", count);

                // Implement heading after testing x and y pos

                mappingTable.put(x_key, x); // Insert values into a dict with x_pos_{count} and y_pos_{count}
                mappingTable.put(y_key, y);
                mappingTable.put(heading_key, heading); // Insert heading into a dict with heading_{count}

                count += 1;

                mappingState = true;
            } else{
                mappingState = false;
            }

            // Current telemetry meant for debug values

            telemetry.addData("X Coord", x);
            telemetry.addData("Y Coord", y);
            telemetry.addData("Heading", heading);
            telemetry.addData("Last X", mapping_x);
            telemetry.addData("Last Y", mapping_y);
            telemetry.addData("Last Heading", mapping_heading);
            telemetry.addData("Test", mappingTable); // Displays complete dict for running mapping values
            telemetry.update();
        }
    }
}