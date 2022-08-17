package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;

import java.util.ArrayList;
import java.util.Collections;

/**
 * Run multiple commands at once, will complete once ALL are complete,
 * To exit when only one is finished see {@link RaceCommand}
 */
public class MultipleCommand extends Command {

	protected ArrayList<Command> commands = new ArrayList<>();

	public MultipleCommand(Command ... commands) {
		Collections.addAll(this.commands, commands);
	}

	@Override
	public void init() {
		for (Command command: commands) {
			command.init();
		}
	}

	@Override
	public void periodic() {
		for (Command command: commands) {
			command.periodic();
		}
	}

	/**
	 * checks if ALL of the actions are done
	 * @return false if any one action is false, returns true if all are true
	 */
	@Override
	public boolean completed() {
		for (Command command: commands) {
			if (!command.completed()) {
				return false;
			}
		}
		return true;
	}

	@Override
	public void shutdown() {
		for (Command command: commands) {
			command.completed();
		}
	}
}
