package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import static org.firstinspires.ftc.teamcode.Utils.ExtraUtils.drawRobotTarget;
import static org.firstinspires.ftc.teamcode.Utils.ExtraUtils.drawRobotTrajectory;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Utils.MathUtils;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.DirectTrajectory;
import org.firstinspires.ftc.teamcode.Robot.ControlConstants;
import org.firstinspires.ftc.teamcode.Math.Controllers.SqrtControl;
import org.firstinspires.ftc.teamcode.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Utils.Stopwatch;

import java.util.ArrayList;

// TODO: Clean this all up, not very understandable right now
public class DriveTrajectory extends Command {
    Drivetrain drivetrain;
    Odometry odometry;

    DirectTrajectory trajectory;
//    TurnOnlyControl turnController;
    SqrtControl angleController = new SqrtControl(ControlConstants.angleControl);
    BasicPID angleController2 = new BasicPID(ControlConstants.AngularVelocityTeleop);
    AngleController angleControl = new AngleController(angleController);
    BasicPID distanceController = new BasicPID(ControlConstants.distanceControl);
    Stopwatch stopwatch = new Stopwatch();

    ArrayList<Pose2d> pastPoses = new ArrayList<Pose2d>();

    public DriveTrajectory(Drivetrain drivetrain, Odometry odometry, DirectTrajectory trajectory) {
        super(drivetrain, odometry);

        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.trajectory = trajectory;

//        this.turnController = new TurnOnlyControl(() -> odometry.getPosition().get(2), trajectory.targetPose(timer.seconds()).getHeading());
    }

    @Override
    public void init() {

    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        if (odometry.getPose().distanceBetween(trajectory.targetPose(stopwatch.seconds())) > 6) {
            stopwatch.setPaused();
        } else {
            stopwatch.setUnpaused();
        }

        double sign = 1;

        double headingError = -MathUtils.normalizedHeadingError(
//                odometry.getPose().angleBetween(trajectory.nextPose(timer.seconds())),
                odometry.getPose().angleBetween(trajectory.targetPose(stopwatch.seconds() + 0.1)),
                odometry.getPose().getHeading()
        );



        if (trajectoryFollowingSign(headingError) < 0) {
            headingError = -MathUtils.normalizedHeadingError(
//                    odometry.getPose().rotate(Math.toRadians(180)).angleBetween(trajectory.nextPose(timer.seconds())),
                    odometry.getPose().rotate(Math.toRadians(180)).angleBetween(trajectory.targetPose(stopwatch.seconds() + 0.1)),
                    odometry.getPose().rotate(Math.toRadians(180)).getHeading()
            );
            sign = -1;
        }

        double turnSpeed = angleControl.calculate(0, headingError);

        double forwardSpeed = -distanceController.calculate(
                0,
                odometry.getPose().distanceBetween(trajectory.targetPose(stopwatch.seconds()))
        ) * sign;
//        forwardSpeed = Range.clip(forwardSpeed, -0.5, 0.5);

        double headingScale = Math.abs(Math.cos(headingError));

//        Vector turnSpeeds = turnController.calculate();
//
//        drivetrain.setPower(
//                forwardSpeed * headingScale + turnSpeeds.get(0),
//                forwardSpeed * headingScale + turnSpeeds.get(1)
//        );
        drivetrain.robotRelative(forwardSpeed * headingScale, turnSpeed);

        pastPoses.add(odometry.getPose());
        Dashboard.packet.put("Current Pose", odometry.getPose());
        Dashboard.packet.put("Target Pose", trajectory.targetPose(stopwatch.seconds()));
        Dashboard.packet.put("Time", stopwatch.seconds());
        Dashboard.packet.put("Distance Between", odometry.getPose().distanceBetween(trajectory.targetPose(stopwatch.seconds())));
        Dashboard.packet.put("Angle Between", odometry.getPose().angleBetween(trajectory.nextPose(stopwatch.seconds())));
        Dashboard.packet.put("Heading Error", headingError);
        Dashboard.packet.put("Trajectory Sign", sign);
        Dashboard.packet.put("Current Heading", odometry.getPose().getHeading());


        drawRobotTarget(trajectory.targetPose(stopwatch.seconds()), Dashboard.packet);
        drawRobotTrajectory(trajectory.getPoses(), "Green", Dashboard.packet);
        drawRobotTrajectory(pastPoses, "Red", Dashboard.packet);
//        drivetrain.setPower(
//                forwardSpeed * headingScale + turnSpeed,
//                forwardSpeed * headingScale - turnSpeed
//        );
    }

    public double trajectoryFollowingSign(double headingError) {
        if (Math.abs(headingError) > Math.toRadians(90))
            return -1;
        else
            return 1;
    }

    @Override
    public boolean completed() {
        return stopwatch.seconds() > trajectory.endTime &&
                odometry.getPose().distanceBetween(trajectory.targetPose(stopwatch.seconds())) < 0.3;
    }

    @Override
    public void shutdown() {
        drivetrain.setPower(0,0);
    }
}
