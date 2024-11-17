package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;

public class CommandMaster {
    Robot robot;

    public CommandMaster(Robot r) {
        robot = r;
    }

    //REGISTER COMMANDS HERE
    public Action locateTargetsCV(CVMaster.EOCVPipeline color) {
        return new LocateTargetsCV(robot, color);
    }

    public Action stopIntake(SampleColors... colors) {
        return new StopIntake(robot, colors);
    }

    public Action waitL2WinchClimbCompletion(Gamepad gamepad) {
        return new L2WinchClimbPoll(robot, gamepad);
    }

    public Action waitForExtension(float target) {
        return new ExtensionPoll(robot, target);
    }
}
