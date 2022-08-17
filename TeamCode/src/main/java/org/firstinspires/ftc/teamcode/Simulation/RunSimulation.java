package org.firstinspires.ftc.teamcode.Simulation;

import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintCommand1;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintSubsystem1;

// TODO: Add actual test cases lol

public class RunSimulation {
    public static void main(String[] args) {
        PrintSubsystem1 printSub1 = new PrintSubsystem1();
        PrintSubsystem1 printSub2 = new PrintSubsystem1();
        PrintCommand1 printCommand1 = new PrintCommand1(printSub1, "Test 1");
        PrintCommand1 printCommand2 = new PrintCommand1(printSub2, "Test 2");

        CommandScheduler scheduler = new CommandScheduler(null, printSub1, printSub2);
        scheduler.initAuto();

//        scheduler.enqueueCommand(printCommand1);
//        scheduler.enqueueCommand(printCommand2);

        scheduler.run();
        scheduler.run();

        scheduler.shutdown();
    }
}
