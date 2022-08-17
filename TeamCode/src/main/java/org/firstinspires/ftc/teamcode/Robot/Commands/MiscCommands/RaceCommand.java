package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;


/**
 * Similar to {@link MultipleCommand}, will run multiple actions at once, but unlike multiple action
 * this command will exit as soon as one command finishes rather than waiting for all.
 */
public class RaceCommand extends MultipleCommand {

	/**
	 * @return true if any actions are done, false if all are incomplete,
	 * otherwise same behavior as the {@link MultipleCommand}
	 */
	@Override
	public boolean completed() {
		for (Command command : commands) {
			if (command.completed()) {
				return true;
			}
		}
		return false;
	}


}
