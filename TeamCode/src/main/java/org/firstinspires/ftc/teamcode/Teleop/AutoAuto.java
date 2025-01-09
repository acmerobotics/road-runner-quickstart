package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.HashMap;
import java.util.Map;

@TeleOp
public class AutoAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d StartPose = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);

        Gamepad currentGamepad1 = gamepad1;
        Gamepad currentGamepad2 = gamepad2;

        boolean mappingState = false; // Switch to make sure no infinite execution

        // Map<String, Double> mappingTable = new HashMap<>();


        double mapping_x = 0.00; // Place holder for last X pos
        double mapping_y = 0.00; // Place holder for last Y pos

        int count = 0; // Count variable for dict/hashmap for data storage, temporary solution? Hopefully better in the future

        waitForStart();

        while (opModeIsActive()){

            double x = drive.pose.position.x; // drive pos x
            double y = drive.pose.position.y; // drive pos y
            if (currentGamepad1.a && !mappingState){
                mapping_x = x;
                mapping_y = y;

                String x_key = "x_pos_" + count;
                String y_key = "y_pos_" + count;

                /* mappingTable.put(x_key, x); // Insert values into a dict with x_pos_{count} and y_pos_{count}
                mappingTable.put(y_key, y); */
                count++;

                mappingState = true;
            } else{
                mappingState = false;
            }

            // Current telemetry meant for debug values

            telemetry.addData("X Coord", x);
            telemetry.addData("Y Coord", y);
            telemetry.addData("Last X", mapping_x);
            telemetry.addData("Last Y", mapping_y);
            telemetry.update();
        }
    }
}