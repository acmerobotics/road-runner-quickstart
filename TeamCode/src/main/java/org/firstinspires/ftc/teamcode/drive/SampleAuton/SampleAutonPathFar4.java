package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SampleAutonPathFar4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive2=new HardwareFile(hardwareMap);
        Pose2d startPose = new Pose2d(-63, 48, 0);

        drive.setPoseEstimate(startPose);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12, 36), 0)
                .build();

        /*Trajectory traj0ring = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(6 , -40),  Math.toRadians(90))
                .build();
*/
        /*Trajectory traj1ring = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(6 , -40),  Math.toRadians(180))
                .build();
*/
        Trajectory traj4ring = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(48 , 36),  Math.toRadians(-135))
                .build();

        Trajectory trajline = drive.trajectoryBuilder(traj4ring.end())
                .splineTo(new Vector2d(0 , 24),  Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectory(traj);
        shooter();
        //0 ring
        drive2.magdown();
        drive.followTrajectory(traj4ring);
        wobbledrop();
        drive.followTrajectory(trajline);
    }
    public static  HardwareFile drive2;
    public void wobbledrop(){
        drive2.grab();
        sleep(100);
        drive2.wobbleArmDown();
        sleep(1000);
        drive2.release();
        sleep(100);
    }
    public void shooter(){
        drive2.shooter(1);
        sleep(2000);
        for(int i=0;i<=3;++i){
            drive2.magup();
            drive2.magup();
            drive2.slapper.setPosition(0);
            sleep(100);
            drive2.slapper.setPosition(0.5);
            sleep(1000);
        }
        drive2.shooter(0);
    }
}
