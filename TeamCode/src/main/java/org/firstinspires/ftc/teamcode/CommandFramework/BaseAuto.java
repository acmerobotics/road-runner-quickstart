package org.firstinspires.ftc.teamcode.CommandFramework;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.visionPipelines.SleeveDetection;

public abstract class BaseAuto extends LinearOpMode {

    protected Robot robot;
    protected TrajectoryBuilder trajectoryBuilder;

    protected SleeveDetection.ParkingPosition parkingPosition = SleeveDetection.ParkingPosition.CENTER;

    @Override
    public void runOpMode() {
        PhotonCore.enable();
        robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2, getTeam());
        setRobotPosition();

        while (!isStopRequested() && !opModeIsActive() && opModeInInit()) {
            parkingPosition = robot.vision.getParkingPosition();
            telemetry.addData("current parking position is: ", parkingPosition);
            telemetry.update();
        }

        robot.vision.frontCam.destroy();


        waitForStart();
        robot.getScheduler().forceCommand(setupAuto(robot.getScheduler()));

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
        }
        robot.shutdown();

    }


    public abstract Command setupAuto(CommandScheduler scheduler);

    public RoadrunnerTrajectoryFollower followRR(Trajectory trajectory) {
        return new RoadrunnerTrajectoryFollower(this.robot, trajectory, robot.dashboard);
    }

    public DelayedCommand delayCommand(double time, Command command) {
        return new DelayedCommand(time,command);
    }

    public MultipleCommand multiCommand(Command... commands) {
        return new MultipleCommand(commands);
    }



    public Delay wait(double seconds) {
        return new Delay(seconds);
    }





    public void setRobotPosition() {

    }

    public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }

    public Team getTeam() { return Team.NOT_ASSIGNED; }


}