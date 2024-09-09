package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldTrajectoryPlanner {

    TrajectoryActionBuilder builder;
    Robot robot;
    public FieldTrajectoryPlanner(Robot robot, Pose2d startingPos) {
        this.builder = robot.drive.getDrive().actionBuilder(startingPos);
        this.robot = robot;
    }

    public FieldTrajectoryPlanner dropSpecimen() {
        builder = builder.strafeToLinearHeading(new Vector2d(5*robot.autoPos.xMult, 40*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));
        return this;
    }

    public FieldTrajectoryPlanner pickNeutral(int number) {
        builder = builder.strafeToLinearHeading(new Vector2d((48+(10*number))*robot.autoPos.yMult, 40*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));
        builder = builder.stopAndAdd(new SleepAction(1));
        return this;
    }

    public FieldTrajectoryPlanner dropNet() {
        builder = builder.strafeToLinearHeading(new Vector2d(52*robot.autoPos.yMult, 52*robot.autoPos.yMult), Math.toRadians(robot.autoPos.yMult > 0 ? 45 : 225));
        builder = builder.stopAndAdd(new SleepAction(1));
        return this;
    }

    public FieldTrajectoryPlanner ascend() {
        builder = builder.strafeToLinearHeading(new Vector2d(23*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0));
        return this;
    }
}
