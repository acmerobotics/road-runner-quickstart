package org.firstinspires.ftc.teamcode.hardware.util;

public class BooleanManager {
	//BooleanManager class aims to clean up the logic behind boolean change states

	//task to be run
	Runnable task;
	boolean formerBoolean = false;

	/**
	 *
	 * @param run the task to be run. Can be initialized with a lambda expression instead of Runnable.
	 *            Ex. () ->{//do stuff}
	 */
	public BooleanManager(Runnable run){
		//runs the run command
		task = run;
	}

	/**
	 *
	 * @param checkValue desired value to be checked. Needs to be within a loop and needs to have access to the desired boolean. Only call ONCE.
	 */
	public void update(boolean checkValue){
		if (checkValue) formerBoolean = true;
		if(formerBoolean && !checkValue){
			formerBoolean = false;

			task.run();
		}
	}

}
