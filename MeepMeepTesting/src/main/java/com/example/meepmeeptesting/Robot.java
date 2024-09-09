package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Robot {

    public  enum AutoPos {
        REDRIGHT (1, -1),
        REDLEFT (-1, -1),
        BLUERIGHT (-1, 1),
        BLUELEFT (1, 1);

        public int xMult;
        public int yMult;

        AutoPos(int xMult, int yMult) {
            this.xMult = xMult;
            this.yMult = yMult;
        }
    }

    public RoadRunnerBotEntity drive;
    public AutoPos autoPos;

    public Robot(RoadRunnerBotEntity drive, AutoPos pos) {
        this.drive = drive;
        this.autoPos = pos;
    }

    public FieldTrajectoryPlanner createTrajectoryPlanner(Pose2d startingPos) {
        return new FieldTrajectoryPlanner(this, startingPos);
    }
}
