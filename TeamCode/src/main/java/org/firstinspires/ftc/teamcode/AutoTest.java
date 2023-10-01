package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTest extends BrainSTEMOpMode {

    public void runOpMode() throws InterruptedException {
        TrajectoryActionBuilder trajectory = new TrajectoryActionBuilder;  //TODO: why is there an error??
        trajectory                //TODO: is this what I'm to do if there are multiple trajectories?
               .lineToXConstantHeading(24)
               .splineTo(new Vector2d(36,0), Math.PI)   //TODO: lineTo vs splineTo??
               .turn(Math.PI / 2)
               .lineToY(48);                //TODO: lineToConstantHeading vs lineTo?
    }
}
