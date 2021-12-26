package org.firstinspires.ftc.teamcode.util.KalmanFilter;

import java.util.Stack;

/**
 * from https://stackoverflow.com/questions/7727919/creating-a-fixed-size-stack
 * @param <T> Type that the stack will hold.
 */
public class SizedStack<T> extends Stack<T> {
	private int maxSize;

	public SizedStack(int size) {
		super();
		this.maxSize = size;
	}

	@Override
	public T push(T object) {
		while (this.size() >= maxSize) {
			this.remove(0);
		}
		return super.push(object);
	}


}
