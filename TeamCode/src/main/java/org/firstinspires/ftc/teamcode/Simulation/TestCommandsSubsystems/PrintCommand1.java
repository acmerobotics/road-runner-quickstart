package org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;

public class PrintCommand1 extends Command {
    PrintSubsystem1 printSubsystem;
    protected String message;

    public PrintCommand1(PrintSubsystem1 printSub, String printMessage) {
        super(printSub);

        printSubsystem = printSub;
        message = printMessage;
    }

    public void init() {
        printSubsystem.print("start: " + message);
    }

    public void periodic() {
        printSubsystem.print("periodic: " + message);
    }

    public boolean completed() {
        return true;
    }

    public void shutdown() {
        printSubsystem.print("shutdown: " + message);
    }
}
