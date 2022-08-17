package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Purepursuit.CurveCalculator;
import org.firstinspires.ftc.teamcode.Purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.DrivePurePursuit;

import java.util.ArrayList;

@Autonomous
public class PurePursuitTest extends BaseAuto {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        ArrayList<CurvePoint> points1 = new ArrayList<>();
        points1.add(new CurvePoint(0,0,0.25));
        points1.add(new CurvePoint(10,10,1, CurveCalculator.FAST_FOLLOW_DIST_IN));
        points1.add(new CurvePoint(20,30,1,CurveCalculator.FAST_FOLLOW_DIST_IN));
        points1.add(new CurvePoint(40,30,1,CurveCalculator.FAST_FOLLOW_DIST_IN));
        points1.add(new CurvePoint(40,60,0.3,CurveCalculator.PRECISE_FOLLOW_DIST_IN));


        ArrayList<CurvePoint> points2 = new ArrayList<>();
        points2.add(new CurvePoint(40,60,0.6,CurveCalculator.FAST_FOLLOW_DIST_IN));
        points2.add(new CurvePoint(40,10,1,CurveCalculator.FAST_FOLLOW_DIST_IN));
        points2.add(new CurvePoint(5,0,0.5,CurveCalculator.PRECISE_FOLLOW_DIST_IN));

        Command auto = new DrivePurePursuit(robot,points1)
                .addNext(turn(Math.toRadians(270)))
                .addNext(new DrivePurePursuit(robot, points2))
                .addNext(turn(0));

        return auto;
    }
}
