package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//This class is to group the repeated sections of autonomous code

@Autonomous
public abstract class Auton_Base extends LinearOpMode {
    //We have an issue with using the same auton for both sides. The start positions are different, and that could lead to potential issues.
    public double slowerVelocity = 8;
    public String ringsDetected;
    public SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //Declaring Trajectories
    public Trajectory traj0;

    public Trajectory trajA1, trajA2, trajA3, trajA4, trajA5, trajA6, trajA7, trajA8;

    public Trajectory trajB1, trajB2, trajB3, trajB4, trajB5, trajB6, trajB7, trajB8;

    public Trajectory trajC1, trajC2, trajC3, trajC4, trajC5, trajC6, trajC7, trajC8;


    public String initHardware_Vision() {
        //Init Hardware
        TFObjectDetector tfod = drive.getTfod();

        // Send telemetry to both the Drivers station and the Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.0, 16.0/9.0);
            telemetry.addLine("TFod Activated!");
            telemetry.update();
            tfod.setClippingMargins(0,0,0,0);
        }

        while (!opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> recognitions = tfod.getRecognitions();
                if (recognitions != null) {
                    telemetry.addData("# Object Detected", recognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : recognitions) {
                        ringsDetected = (String)recognition.getLabel();
                        telemetry.addData(String.format("label (%d)", i), (String)recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                    if (recognitions.size() == 0)
                        ringsDetected = "None";
//                    if(recognitions.size() != 0)
//                    {
//                        if (tfod != null)
//                            tfod.shutdown();
//                        break;
//                    }

                }
//                else {
//                    ringsDetected = "None";
//                }
            }
        }

        if (ringsDetected == null)
            ringsDetected = "None";

        return ringsDetected;
    }

    public void pathShoot() {
        drive.shooterMotor.setVelocity(drive.targetVel);
        drive.loaderPos = 0.0;
        drive.moveTo("Away");
        drive.followTrajectory(traj0);
        drive.shootRings(3);
    }

}
