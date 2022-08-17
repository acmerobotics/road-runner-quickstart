package org.firstinspires.ftc.teamcode.CommandFramework;

import java.util.ArrayList;
import java.util.Collections;

public abstract class Command {
    protected Command nextCommand = null;

    public void setNext(Command command) { nextCommand = command; }

    public Command getNext() { return nextCommand; }

    public Command addNext(Command command) {
        Command commandNode = this;
        while (commandNode.getNext() != null)
            commandNode = commandNode.getNext();

        commandNode.setNext(command);

        return this;
    }

    protected ArrayList<Subsystem> dependencies = new ArrayList<>();

    public ArrayList<Subsystem> getDependencies() {
        return dependencies;
    }

    public Command(Subsystem ...subsystems) {
        Collections.addAll(dependencies, subsystems);
    }

    public abstract void init();

    public abstract void periodic();

    public abstract boolean completed();

    public abstract void shutdown();
}
